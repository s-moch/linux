// SPDX-License-Identifier: GPL-2.0+

#include "saa716x_mod.h"

#include "saa716x_gpio_reg.h"
#include "saa716x_greg_reg.h"
#include "saa716x_msi_reg.h"

#include "saa716x_adap.h"
#include "saa716x_boot.h"
#include "saa716x_i2c.h"
#include "saa716x_pci.h"
#include "saa716x_hybrid.h"
#include "saa716x_gpio.h"
#include "saa716x_priv.h"

#include "tda1004x.h"
#include "tda827x.h"

unsigned int int_type = 1;
module_param(int_type, int, 0644);
MODULE_PARM_DESC(int_type, "select Interrupt Handler type: 0=INT-A, 1=MSI. default: MSI mode");

#define DRIVER_NAME	"SAA716x Hybrid"

static int saa716x_hybrid_pci_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
	struct saa716x_dev *saa716x;
	int err = 0;

	saa716x = kzalloc(sizeof(struct saa716x_dev), GFP_KERNEL);
	if (saa716x == NULL) {
		err = -ENOMEM;
		goto fail0;
	}

	saa716x->int_type	= int_type;
	saa716x->pdev		= pdev;
	saa716x->module		= THIS_MODULE;
	saa716x->config		= (struct saa716x_config *) pci_id->driver_data;

	err = saa716x_pci_init(saa716x);
	if (err) {
		pci_err(saa716x->pdev, "PCI Initialization failed");
		goto fail1;
	}

	err = saa716x_cgu_init(saa716x);
	if (err) {
		pci_err(saa716x->pdev, "CGU Init failed");
		goto fail1;
	}

	err = saa716x_jetpack_init(saa716x);
	if (err) {
		pci_err(saa716x->pdev, "Jetpack core Initialization failed");
		goto fail2;
	}

	err = saa716x_i2c_init(saa716x);
	if (err) {
		pci_err(saa716x->pdev, "I2C Initialization failed");
		goto fail3;
	}

	saa716x_gpio_init(saa716x);

	/* enable decoders on 7162 */
	if (pdev->device == SAA7162) {
		saa716x_gpio_set_output(saa716x, 24);
		saa716x_gpio_set_output(saa716x, 25);

		saa716x_gpio_write(saa716x, 24, 0);
		saa716x_gpio_write(saa716x, 25, 0);

		msleep(10);

		saa716x_gpio_write(saa716x, 24, 1);
		saa716x_gpio_write(saa716x, 25, 1);
	}

	err = saa716x_dvb_init(saa716x);
	if (err) {
		pci_err(saa716x->pdev, "DVB initialization failed");
		goto fail4;
	}

	return 0;

fail4:
	saa716x_dvb_exit(saa716x);
fail3:
	saa716x_i2c_exit(saa716x);
fail2:
	saa716x_pci_exit(saa716x);
fail1:
	kfree(saa716x);
fail0:
	return err;
}

static void saa716x_hybrid_pci_remove(struct pci_dev *pdev)
{
	struct saa716x_dev *saa716x = pci_get_drvdata(pdev);

	saa716x_dvb_exit(saa716x);
	saa716x_i2c_exit(saa716x);
	saa716x_pci_exit(saa716x);
	kfree(saa716x);
}

static irqreturn_t saa716x_hybrid_pci_irq(int irq, void *dev_id)
{
	struct saa716x_dev *saa716x	= (struct saa716x_dev *) dev_id;

	u32 stat_h, stat_l, mask_h, mask_l;

	stat_l = SAA716x_EPRD(MSI, MSI_INT_STATUS_L);
	stat_h = SAA716x_EPRD(MSI, MSI_INT_STATUS_H);
	mask_l = SAA716x_EPRD(MSI, MSI_INT_ENA_L);
	mask_h = SAA716x_EPRD(MSI, MSI_INT_ENA_H);

	pci_dbg(saa716x->pdev, "MSI STAT L=<%02x> H=<%02x>, CTL L=<%02x> H=<%02x>",
		stat_l, stat_h, mask_l, mask_h);

	if (!((stat_l & mask_l) || (stat_h & mask_h)))
		return IRQ_NONE;

	if (stat_l)
		SAA716x_EPWR(MSI, MSI_INT_STATUS_CLR_L, stat_l);
	if (stat_h)
		SAA716x_EPWR(MSI, MSI_INT_STATUS_CLR_H, stat_h);

	if (stat_l & MSI_INT_TAGACK_FGPI_0)
		tasklet_schedule(&saa716x->fgpi[0].tasklet);
	if (stat_l & MSI_INT_TAGACK_FGPI_1)
		tasklet_schedule(&saa716x->fgpi[1].tasklet);
	if (stat_l & MSI_INT_TAGACK_FGPI_2)
		tasklet_schedule(&saa716x->fgpi[2].tasklet);
	if (stat_l & MSI_INT_TAGACK_FGPI_3)
		tasklet_schedule(&saa716x->fgpi[3].tasklet);

	return IRQ_HANDLED;
}

/*
 * Twinhan/Azurewave VP-6090
 * DVB-S Frontend: 2x MB86A16 - not supported yet
 * DVB-T Frontend: 2x TDA10046 + TDA8275
 */
#define SAA716x_MODEL_TWINHAN_VP6090	"Twinhan/Azurewave VP-6090"
#define SAA716x_DEV_TWINHAN_VP6090	"2xDVB-S + 2xDVB-T + 2xAnalog"

static int tda1004x_vp6090_request_firmware(struct dvb_frontend *fe,
					      const struct firmware **fw,
					      char *name)
{
	struct saa716x_adapter *adapter = fe->dvb->priv;

	return request_firmware(fw, name, &adapter->saa716x->pdev->dev);
}

static const struct tda1004x_config tda1004x_vp6090_config = {
	.demod_address		= 0x8,
	.invert			= 0,
	.invert_oclk		= 0,
	.xtal_freq		= TDA10046_XTAL_4M,
	.agc_config		= TDA10046_AGC_DEFAULT,
	.if_freq		= TDA10046_FREQ_3617,
	.request_firmware	= tda1004x_vp6090_request_firmware,
};

static int saa716x_vp6090_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	struct saa716x_i2c *i2c = &saa716x->i2c[count];

	pci_dbg(saa716x->pdev, "Adapter (%d) SAA716x frontend Init", count);
	pci_dbg(saa716x->pdev, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);
	pci_dbg(saa716x->pdev, "Adapter (%d) Power ON", count);

	saa716x_gpio_set_output(saa716x, 11);
	saa716x_gpio_set_output(saa716x, 10);
	saa716x_gpio_write(saa716x, 11, 1);
	saa716x_gpio_write(saa716x, 10, 1);
	msleep(100);

	adapter->fe = tda10046_attach(&tda1004x_vp6090_config, &i2c->i2c_adapter);
	if (adapter->fe == NULL) {
		pci_err(saa716x->pdev, "Frontend attach failed");
		return -ENODEV;
	}
	pci_dbg(saa716x->pdev, "Done!");
	return 0;
}

static const struct saa716x_config saa716x_vp6090_config = {
	.model_name		= SAA716x_MODEL_TWINHAN_VP6090,
	.dev_type		= SAA716x_DEV_TWINHAN_VP6090,
	.adapters		= 1,
	.frontend_attach	= saa716x_vp6090_frontend_attach,
	.irq_handler		= saa716x_hybrid_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
};

/*
 * NXP Reference design (Atlantis)
 * 2x DVB-T Frontend: 2x TDA10046
 * Analog Decoder: 2x Internal
 */
#define SAA716x_MODEL_NXP_ATLANTIS	"Atlantis reference board"
#define SAA716x_DEV_NXP_ATLANTIS	"2x DVB-T + 2x Analog"

static int tda1004x_atlantis_request_firmware(struct dvb_frontend *fe,
					      const struct firmware **fw,
					      char *name)
{
	struct saa716x_adapter *adapter = fe->dvb->priv;

	return request_firmware(fw, name, &adapter->saa716x->pdev->dev);
}

static const struct tda1004x_config tda1004x_atlantis_config = {
	.demod_address		= 0x8,
	.invert			= 0,
	.invert_oclk		= 0,
	.xtal_freq		= TDA10046_XTAL_16M,
	.agc_config		= TDA10046_AGC_TDA827X,
	.if_freq		= TDA10046_FREQ_045,
	.request_firmware	= tda1004x_atlantis_request_firmware,
	.tuner_address          = 0x60,
};

static struct tda827x_config tda827x_atlantis_config = {
	.init		= NULL,
	.sleep		= NULL,
	.config		= 0,
	.switch_addr	= 0,
	.agcf		= NULL,
};

static int saa716x_atlantis_frontend_attach(struct saa716x_adapter *adapter,
					    int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	struct saa716x_i2c *i2c;
	u8 i2c_buf[3] = { 0x05, 0x23, 0x01 }; /* activate the silent I2C bus */
	struct i2c_msg msg = {
		.addr  = 0x42 >> 1,
		.flags = 0,
		.buf   = i2c_buf,
		.len   = sizeof(i2c_buf)
	};

	if (count < saa716x->config->adapters) {
		u32 reset_gpio;

		pci_dbg(saa716x->pdev, "Adapter (%d) SAA716x frontend Init",
			count);
		pci_dbg(saa716x->pdev, "Adapter (%d) Device ID=%02x", count,
			saa716x->pdev->subsystem_device);

		if (count == 0) {
			reset_gpio = 14;
			i2c = &saa716x->i2c[SAA716x_I2C_BUS_A];
		} else {
			reset_gpio = 15;
			i2c = &saa716x->i2c[SAA716x_I2C_BUS_B];
		}

		/* activate the silent I2C bus */
		i2c_transfer(&i2c->i2c_adapter, &msg, 1);

		saa716x_gpio_set_output(saa716x, reset_gpio);

		/* Reset the demodulator */
		saa716x_gpio_write(saa716x, reset_gpio, 1);
		msleep(10);
		saa716x_gpio_write(saa716x, reset_gpio, 0);
		msleep(10);
		saa716x_gpio_write(saa716x, reset_gpio, 1);
		msleep(10);

		adapter->fe = tda10046_attach(&tda1004x_atlantis_config,
					      &i2c->i2c_adapter);
		if (adapter->fe == NULL)
			goto exit;

		pci_dbg(saa716x->pdev,
			"found TDA10046 DVB-T frontend @0x%02x",
			tda1004x_atlantis_config.demod_address);

		if (dvb_attach(tda827x_attach, adapter->fe,
			       tda1004x_atlantis_config.tuner_address,
			       &i2c->i2c_adapter, &tda827x_atlantis_config)) {
			pci_dbg(saa716x->pdev, "found TDA8275 tuner @0x%02x",
				tda1004x_atlantis_config.tuner_address);
		} else {
			goto exit;
		}

		pci_dbg(saa716x->pdev, "Done!");
		return 0;
	}

exit:
	pci_err(saa716x->pdev, "Frontend attach failed");
	return -ENODEV;
}

static const struct saa716x_config saa716x_atlantis_config = {
	.model_name		= SAA716x_MODEL_NXP_ATLANTIS,
	.dev_type		= SAA716x_DEV_NXP_ATLANTIS,
	.adapters		= 2,
	.frontend_attach	= saa716x_atlantis_frontend_attach,
	.irq_handler		= saa716x_hybrid_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
	.adap_config		= {
		{
			/* Adapter 0 */
			.ts_vp   = 3,
			.ts_fgpi = 3
		},
		{
			/* Adapter 1 */
			.ts_vp   = 6,
			.ts_fgpi = 0
		}
	}
};

/*
 * NXP Reference design (NEMO)
 * DVB-T Frontend: 1x TDA10046 + TDA8275
 * Analog Decoder: External SAA7136
 */
#define SAA716x_MODEL_NXP_NEMO		"NEMO reference board"
#define SAA716x_DEV_NXP_NEMO		"DVB-T + Analog"

static int tda1004x_nemo_request_firmware(struct dvb_frontend *fe,
					  const struct firmware **fw,
					  char *name)
{
	struct saa716x_adapter *adapter = fe->dvb->priv;

	return request_firmware(fw, name, &adapter->saa716x->pdev->dev);
}

static const struct tda1004x_config tda1004x_nemo_config = {
	.demod_address		= 0x8,
	.invert			= 0,
	.invert_oclk		= 0,
	.xtal_freq		= TDA10046_XTAL_16M,
	.agc_config		= TDA10046_AGC_TDA827X,
	.if_freq		= TDA10046_FREQ_045,
	.request_firmware	= tda1004x_nemo_request_firmware,
	.tuner_address          = 0x60,
};

static struct tda827x_config tda827x_nemo_config = {
	.init		= NULL,
	.sleep		= NULL,
	.config		= 0,
	.switch_addr	= 0,
	.agcf		= NULL,
};

static int saa716x_nemo_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	struct saa716x_i2c *demod_i2c = &saa716x->i2c[SAA716x_I2C_BUS_B];
	struct saa716x_i2c *tuner_i2c = &saa716x->i2c[SAA716x_I2C_BUS_A];


	if (count  == 0) {
		pci_dbg(saa716x->pdev, "Adapter (%d) SAA716x frontend Init", count);
		pci_dbg(saa716x->pdev, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);
		pci_dbg(saa716x->pdev, "Adapter (%d) Power ON", count);

		/* GPIO 26 controls a +15dB gain */
		saa716x_gpio_set_output(saa716x, 26);
		saa716x_gpio_write(saa716x, 26, 0);

		saa716x_gpio_set_output(saa716x, 14);

		/* Reset the demodulator */
		saa716x_gpio_write(saa716x, 14, 1);
		msleep(10);
		saa716x_gpio_write(saa716x, 14, 0);
		msleep(10);
		saa716x_gpio_write(saa716x, 14, 1);
		msleep(10);

		adapter->fe = tda10046_attach(&tda1004x_nemo_config,
					      &demod_i2c->i2c_adapter);
		if (adapter->fe) {
			pci_dbg(saa716x->pdev, "found TDA10046 DVB-T frontend @0x%02x",
				tda1004x_nemo_config.demod_address);

		} else {
			goto exit;
		}
		if (dvb_attach(tda827x_attach, adapter->fe,
			       tda1004x_nemo_config.tuner_address,
			       &tuner_i2c->i2c_adapter, &tda827x_nemo_config)) {
			pci_dbg(saa716x->pdev, "found TDA8275 tuner @0x%02x",
				tda1004x_nemo_config.tuner_address);
		} else {
			goto exit;
		}
		pci_dbg(saa716x->pdev, "Done!");
	}

	return 0;
exit:
	pci_err(saa716x->pdev, "Frontend attach failed");
	return -ENODEV;
}

static const struct saa716x_config saa716x_nemo_config = {
	.model_name		= SAA716x_MODEL_NXP_NEMO,
	.dev_type		= SAA716x_DEV_NXP_NEMO,
	.adapters		= 1,
	.frontend_attach	= saa716x_nemo_frontend_attach,
	.irq_handler		= saa716x_hybrid_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,

	.adap_config		= {
		{
			/* Adapter 0 */
			.ts_vp   = 3,
			.ts_fgpi = 3
		}
	}
};

static const struct pci_device_id saa716x_hybrid_pci_table[] = {

	MAKE_ENTRY(TWINHAN_TECHNOLOGIES, TWINHAN_VP_6090, SAA7162, &saa716x_vp6090_config),
	MAKE_ENTRY(KWORLD, KWORLD_DVB_T_PE310, SAA7162, &saa716x_atlantis_config),
	MAKE_ENTRY(NXP_REFERENCE_BOARD, PCI_ANY_ID, SAA7162, &saa716x_atlantis_config),
	MAKE_ENTRY(NXP_REFERENCE_BOARD, PCI_ANY_ID, SAA7160, &saa716x_nemo_config),
	{ }
};
MODULE_DEVICE_TABLE(pci, saa716x_hybrid_pci_table);

static struct pci_driver saa716x_hybrid_pci_driver = {
	.name			= DRIVER_NAME,
	.id_table		= saa716x_hybrid_pci_table,
	.probe			= saa716x_hybrid_pci_probe,
	.remove			= saa716x_hybrid_pci_remove,
};

static int __init saa716x_hybrid_init(void)
{
	return pci_register_driver(&saa716x_hybrid_pci_driver);
}

static void __exit saa716x_hybrid_exit(void)
{
	return pci_unregister_driver(&saa716x_hybrid_pci_driver);
}

module_init(saa716x_hybrid_init);
module_exit(saa716x_hybrid_exit);

MODULE_DESCRIPTION("SAA716x Hybrid driver");
MODULE_AUTHOR("Manu Abraham");
MODULE_LICENSE("GPL");
