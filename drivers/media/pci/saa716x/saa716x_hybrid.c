#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/mutex.h>

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <linux/kmod.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/device.h>

#include <asm/irq.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include <linux/i2c.h>

#include "saa716x_reg.h"
#include "saa716x_adap.h"
#include "saa716x_i2c.h"
#include "saa716x_msi.h"
#include "saa716x_hybrid.h"
#include "saa716x_gpio.h"
#include "saa716x_reg.h"
#include "saa716x_rom.h"
#include "saa716x_spi.h"
#include "saa716x_priv.h"

#include "zl10353.h"
#include "mb86a16.h"
#include "tda1004x.h"

unsigned int verbose;
module_param(verbose, int, 0644);
MODULE_PARM_DESC(verbose, "verbose startup messages, default is 1 (yes)");

unsigned int int_type;
module_param(int_type, int, 0644);
MODULE_PARM_DESC(int_type, "force Interrupt Handler type: 0=INT-A, 1=MSI, 2=MSI-X. default INT-A mode");

#define DRIVER_NAME	"SAA716x Hybrid"

static int __devinit saa716x_hybrid_pci_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
	struct saa716x_dev *saa716x;
	int err = 0;
	u32 sts;

	saa716x = kzalloc(sizeof (struct saa716x_dev), GFP_KERNEL);
	if (saa716x == NULL) {
		printk(KERN_ERR "saa716x_hybrid_pci_probe ERROR: out of memory\n");
		err = -ENOMEM;
		goto fail0;
	}

	saa716x->verbose	= verbose;
	saa716x->int_type	= int_type;
	saa716x->pdev		= pdev;
	saa716x->config		= (struct saa716x_config *) pci_id->driver_data;

	err = saa716x_pci_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x PCI Initialization failed");
		goto fail1;
	}

	err = saa716x_cgu_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x CGU Init failed");
		goto fail1;
	}

	err = saa716x_core_boot(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x Core Boot failed");
		goto fail2;
	}
	dprintk(SAA716x_DEBUG, 1, "SAA716x Core Boot Success");

	err = saa716x_msi_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x MSI Init failed");
		goto fail2;
	}

	err = saa716x_jetpack_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x Jetpack core Initialization failed");
		goto fail1;
	}

	pci_read_config_dword(pdev, 0x06, &sts);

	err = saa716x_i2c_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x I2C Initialization failed");
		goto fail3;
	}

	err = saa716x_dump_eeprom(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x EEPROM dump failed");
	}

	err = saa716x_eeprom_data(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x EEPROM dump failed");
	}

	err = saa716x_dvb_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x DVB initialization failed");
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

static void __devexit saa716x_hybrid_pci_remove(struct pci_dev *pdev)
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

	if (unlikely(saa716x == NULL)) {
		printk("%s: saa716x=NULL", __func__);
		return IRQ_NONE;
	}

	stat_l = SAA716x_EPRD(MSI, MSI_INT_STATUS_L);
	stat_h = SAA716x_EPRD(MSI, MSI_INT_STATUS_H);
	mask_l = SAA716x_EPRD(MSI, MSI_INT_ENA_L);
	mask_h = SAA716x_EPRD(MSI, MSI_INT_ENA_H);

	dprintk(SAA716x_DEBUG, 1, "MSI STAT L=<%02x> H=<%02x>, CTL L=<%02x> H=<%02x>",
		stat_l, stat_h, mask_l, mask_h);

	if (!((stat_l & mask_l) || (stat_h & mask_h)))
		return IRQ_NONE;

	if (stat_l)
		SAA716x_EPWR(MSI, MSI_INT_STATUS_CLR_L, stat_l);

	if (stat_h)
		SAA716x_EPWR(MSI, MSI_INT_STATUS_CLR_H, stat_h);

	saa716x_msi_event(saa716x, stat_l, stat_h);
#if 0
	dprintk(SAA716x_DEBUG, 1, "VI STAT 0=<%02x> 1=<%02x>, CTL 1=<%02x> 2=<%02x>",
		SAA716x_EPRD(VI0, INT_STATUS),
		SAA716x_EPRD(VI1, INT_STATUS),
		SAA716x_EPRD(VI0, INT_ENABLE),
		SAA716x_EPRD(VI1, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "FGPI STAT 0=<%02x> 1=<%02x>, CTL 1=<%02x> 2=<%02x>",
		SAA716x_EPRD(FGPI0, INT_STATUS),
		SAA716x_EPRD(FGPI1, INT_STATUS),
		SAA716x_EPRD(FGPI0, INT_ENABLE),
		SAA716x_EPRD(FGPI0, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "FGPI STAT 2=<%02x> 3=<%02x>, CTL 2=<%02x> 3=<%02x>",
		SAA716x_EPRD(FGPI2, INT_STATUS),
		SAA716x_EPRD(FGPI3, INT_STATUS),
		SAA716x_EPRD(FGPI2, INT_ENABLE),
		SAA716x_EPRD(FGPI3, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "AI STAT 0=<%02x> 1=<%02x>, CTL 0=<%02x> 1=<%02x>",
		SAA716x_EPRD(AI0, AI_STATUS),
		SAA716x_EPRD(AI1, AI_STATUS),
		SAA716x_EPRD(AI0, AI_CTL),
		SAA716x_EPRD(AI1, AI_CTL));

	dprintk(SAA716x_DEBUG, 1, "I2C STAT 0=<%02x> 1=<%02x>, CTL 0=<%02x> 1=<%02x>",
		SAA716x_EPRD(I2C_A, INT_STATUS),
		SAA716x_EPRD(I2C_B, INT_STATUS),
		SAA716x_EPRD(I2C_A, INT_ENABLE),
		SAA716x_EPRD(I2C_B, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "DCS STAT=<%02x>, CTL=<%02x>",
		SAA716x_EPRD(DCS, DCSC_INT_STATUS),
		SAA716x_EPRD(DCS, DCSC_INT_ENABLE));
#endif

	return IRQ_HANDLED;
}


/*
 * Twinhan/Azurewave VP-6090
 * DVB-S Frontend: 2x MB86A16
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

static struct tda1004x_config tda1004x_vp6090_config = {
	.demod_address		= 0x8,
	.invert			= 0,
	.invert_oclk		= 0,
	.xtal_freq		= TDA10046_XTAL_4M,
	.agc_config		= TDA10046_AGC_DEFAULT,
	.if_freq		= TDA10046_FREQ_3617,
	.request_firmware	= tda1004x_vp6090_request_firmware,
};

static int load_config_vp6090(struct saa716x_dev *saa716x)
{
	int ret = 0;

	return ret;
}

static int vp6090_dvbs_set_voltage(struct dvb_frontend *fe, fe_sec_voltage_t voltage)
{
	struct saa716x_dev *saa716x = fe->dvb->priv;

	switch (voltage) {
	case SEC_VOLTAGE_13:
		dprintk(SAA716x_ERROR, 1, "Polarization=[13V]");
		break;
	case SEC_VOLTAGE_18:
		dprintk(SAA716x_ERROR, 1, "Polarization=[18V]");
		break;
	case SEC_VOLTAGE_OFF:
		dprintk(SAA716x_ERROR, 1, "Frontend (dummy) POWERDOWN");
		break;
	default:
		dprintk(SAA716x_ERROR, 1, "Invalid = (%d)", (u32 ) voltage);
		return -EINVAL;
	}

	return 0;
}

struct mb86a16_config vp6090_mb86a16_config = {
	.demod_address	= 0x08,
	.set_voltage	= vp6090_dvbs_set_voltage,
};

static int saa716x_vp6090_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	struct saa716x_i2c *i2c = &saa716x->i2c[count];

	dprintk(SAA716x_ERROR, 1, "Adapter (%d) SAA716x frontend Init", count);
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);

	dprintk(SAA716x_ERROR, 1, "Adapter (%d) Power ON", count);

	saa716x_gpio_write(saa716x, GPIO_11, 1);
	saa716x_gpio_write(saa716x, GPIO_10, 1);
	msleep(100);
#if 0
	dprintk(SAA716x_ERROR, 1, "Probing for MB86A16 (DVB-S/DSS)");
	adapter->fe = mb86a16_attach(&vp6090_mb86a16_config, &i2c->i2c_adapter);
	if (adapter->fe) {
		dprintk(SAA716x_ERROR, 1, "found MB86A16 DVB-S/DSS frontend @0x%02x",
			vp6090_mb86a16_config.demod_address);

	} else {
		goto exit;
	}
#endif
	adapter->fe = tda10046_attach(&tda1004x_vp6090_config, &i2c->i2c_adapter);
	if (adapter->fe == NULL) {
		dprintk(SAA716x_ERROR, 1, "A frontend driver was not found for [%04x:%04x subsystem [%04x:%04x]\n",
			saa716x->pdev->vendor,
			saa716x->pdev->device,
			saa716x->pdev->subsystem_vendor,
			saa716x->pdev->subsystem_device);
	} else {
		if (dvb_register_frontend(&adapter->dvb_adapter, adapter->fe)) {
			dprintk(SAA716x_ERROR, 1, "Frontend registration failed!\n");
			dvb_frontend_detach(adapter->fe);
			adapter->fe = NULL;
			goto exit;
		}
	}

	dprintk(SAA716x_ERROR, 1, "Done!");
	return 0;
exit:
	dprintk(SAA716x_ERROR, 1, "Frontend attach failed");
	return -ENODEV;
}

static struct saa716x_config saa716x_vp6090_config = {
	.model_name		= SAA716x_MODEL_TWINHAN_VP6090,
	.dev_type		= SAA716x_DEV_TWINHAN_VP6090,
	.boot_mode		= SAA716x_EXT_BOOT,
	.load_config		= &load_config_vp6090,
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

static struct tda1004x_config tda1004x_atlantis_config = {
	.demod_address		= 0x8,
	.invert			= 0,
	.invert_oclk		= 0,
	.xtal_freq		= TDA10046_XTAL_4M,
	.agc_config		= TDA10046_AGC_DEFAULT,
	.if_freq		= TDA10046_FREQ_3617,
	.request_firmware	= tda1004x_atlantis_request_firmware,
};

static int load_config_atlantis(struct saa716x_dev *saa716x)
{
	int ret = 0;
	return ret;
}

static int saa716x_atlantis_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	struct saa716x_i2c *i2c = &saa716x->i2c[count];

	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) SAA716x frontend Init", count);
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);
	dprintk(SAA716x_ERROR, 1, "Adapter (%d) Power ON", count);
	saa716x_gpio_write(saa716x, GPIO_14, 1);
	msleep(100);

	adapter->fe = tda10046_attach(&tda1004x_atlantis_config, &i2c->i2c_adapter);
	if (adapter->fe == NULL) {
		dprintk(SAA716x_ERROR, 1, "A frontend driver was not found for [%04x:%04x subsystem [%04x:%04x]\n",
			saa716x->pdev->vendor,
			saa716x->pdev->device,
			saa716x->pdev->subsystem_vendor,
			saa716x->pdev->subsystem_device);
	} else {
		if (dvb_register_frontend(&adapter->dvb_adapter, adapter->fe)) {
			dprintk(SAA716x_ERROR, 1, "Frontend registration failed!\n");
			dvb_frontend_detach(adapter->fe);
			adapter->fe = NULL;
		}
	}

	return -ENODEV;
}

static struct saa716x_config saa716x_atlantis_config = {
	.model_name		= SAA716x_MODEL_NXP_ATLANTIS,
	.dev_type		= SAA716x_DEV_NXP_ATLANTIS,
	.boot_mode		= SAA716x_EXT_BOOT,
	.load_config		= &load_config_atlantis,
	.adapters		= 2,
	.frontend_attach	= saa716x_atlantis_frontend_attach,
	.irq_handler		= saa716x_hybrid_pci_irq,
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

static struct tda1004x_config tda1004x_nemo_config = {
	.demod_address		= 0x8,
	.invert			= 0,
	.invert_oclk		= 0,
	.xtal_freq		= TDA10046_XTAL_4M,
	.agc_config		= TDA10046_AGC_DEFAULT,
	.if_freq		= TDA10046_FREQ_3617,
	.request_firmware	= tda1004x_nemo_request_firmware,
};

static int load_config_nemo(struct saa716x_dev *saa716x)
{
    int ret = 0;
    return ret;
}

static int saa716x_nemo_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	struct saa716x_i2c *i2c = &saa716x->i2c[1];

	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) SAA716x frontend Init", count);
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);

	dprintk(SAA716x_ERROR, 1, "Adapter (%d) Power ON", count);
	saa716x_gpio_write(saa716x, GPIO_14, 1);
	msleep(100);
	adapter->fe = tda10046_attach(&tda1004x_nemo_config, &i2c->i2c_adapter);
	if (adapter->fe == NULL) {
		dprintk(SAA716x_ERROR, 1, "A frontend driver was not found for [%04x:%04x subsystem [%04x:%04x]\n",
			saa716x->pdev->vendor,
			saa716x->pdev->device,
			saa716x->pdev->subsystem_vendor,
			saa716x->pdev->subsystem_device);
	} else {
		if (dvb_register_frontend(&adapter->dvb_adapter, adapter->fe)) {
			dprintk(SAA716x_ERROR, 1, "Frontend registration failed!\n");
			dvb_frontend_detach(adapter->fe);
			adapter->fe = NULL;
		}
	}

	return 0;
}

static struct saa716x_config saa716x_nemo_config = {
	.model_name		= SAA716x_MODEL_NXP_NEMO,
	.dev_type		= SAA716x_DEV_NXP_NEMO,
	.boot_mode		= SAA716x_EXT_BOOT,
	.load_config		= &load_config_nemo,
	.adapters		= 1,
	.frontend_attach	= saa716x_nemo_frontend_attach,
	.irq_handler		= saa716x_hybrid_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,
};


#define SAA716x_MODEL_AVERMEDIA_HC82	"Avermedia HC82 Express-54"
#define SAA716x_DEV_AVERMEDIA_HC82	"DVB-T + Analog"

static int load_config_averhc82(struct saa716x_dev *saa716x)
{
	int ret = 0;
	return ret;
}

#if 0
static struct zl10353_config saa716x_averhc82_zl10353_config = {
	.demod_address		= 0x1f,
	.adc_clock		= 450560,
	.if2			= 361667,
	.no_tuner		= 1,
	.parallel_ts		= 1,
};
#endif

static int saa716x_averhc82_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
//	struct saa716x_i2c *i2c = &saa716x->i2c[count];

	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) SAA716x frontend Init", count);
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);

//	adapter->fe = zl10353_attach(&saa716x_averhc82_zl10353_config, &i2c->i2c_adapter);
	if (adapter->fe == NULL) {
		dprintk(SAA716x_ERROR, 1, "No Frontend found");
		return -ENODEV;
	} else {
		dprintk(SAA716x_ERROR, 1, "Adapter (%d) ZL10353 demodulator succesfully attached", count);
		if (dvb_register_frontend(&adapter->dvb_adapter, adapter->fe)) {
			dprintk(SAA716x_ERROR, 1, "ERROR: Frontend registration failed");

			if (adapter->fe->ops.release)
				adapter->fe->ops.release(adapter->fe);

			adapter->fe = NULL;
			return -ENODEV;
		}
	}

	return 0;
}

static struct saa716x_config saa716x_averhc82_config = {
	.model_name		= SAA716x_MODEL_AVERMEDIA_HC82,
	.dev_type		= SAA716x_DEV_AVERMEDIA_HC82,
	.boot_mode		= SAA716x_EXT_BOOT,
	.load_config		= &load_config_averhc82,
	.adapters		= 1,
	.demodulator_addr	= 0x1f,
	.tuner_addr		= 0xc0,
	.decoder_addr		= 0x42,
	.frontend_attach	= saa716x_averhc82_frontend_attach,
	.irq_handler		= saa716x_hybrid_pci_irq,

	.adap_config		= {
		{ /* Adapter 0 */
			.power_ctl	= GPIO_06 | GPIO_15,
			.reset_ctl	= GPIO_05,
		}
	}
};

#define SAA716x_MODEL_AVERMEDIA_H788	"Avermedia H788"
#define SAA716x_DEV_AVERMEDIA_H788	"DVB-T + Analaog"

static int load_config_averh788(struct saa716x_dev *saa716x)
{
	int ret = 0;
	return ret;
}

static int saa716x_averh88_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;

	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) SAA716x frontend Init", count);
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);

	return -ENODEV;
}

static struct saa716x_config saa716x_averh788_config = {
	.model_name		= SAA716x_MODEL_AVERMEDIA_H788,
	.dev_type		= SAA716x_DEV_AVERMEDIA_H788,
	.boot_mode		= SAA716x_EXT_BOOT,
	.load_config		= &load_config_averh788,
	.adapters		= 1,
	.demodulator_addr	= 0x1f,
	.tuner_addr		= 0xc0,
	.decoder_addr		= 0x42,
	.frontend_attach	= saa716x_averh88_frontend_attach,
	.irq_handler		= saa716x_hybrid_pci_irq,

	.adap_config		= {
		{ /* Adapter 0 */
			.power_ctl	= GPIO_14,
			.reset_ctl	= GPIO_15,
		}
	}

};

static struct pci_device_id saa716x_hybrid_pci_table[] = {

	MAKE_ENTRY(TWINHAN_TECHNOLOGIES, TWINHAN_VP_6090, SAA7162, &saa716x_vp6090_config),
	MAKE_ENTRY(NXP_REFERENCE_BOARD, PCI_ANY_ID, SAA7162, &saa716x_atlantis_config),
	MAKE_ENTRY(NXP_REFERENCE_BOARD, PCI_ANY_ID, SAA7160, &saa716x_nemo_config),
	MAKE_ENTRY(AVERMEDIA, AVERMEDIA_HC82, SAA7160, &saa716x_averhc82_config),
	MAKE_ENTRY(AVERMEDIA, AVERMEDIA_H788, SAA7160, &saa716x_averh788_config),
	{ }
};
MODULE_DEVICE_TABLE(pci, saa716x_hybrid_pci_table);

static struct pci_driver saa716x_hybrid_pci_driver = {
	.name			= DRIVER_NAME,
	.id_table		= saa716x_hybrid_pci_table,
	.probe			= saa716x_hybrid_pci_probe,
	.remove			= saa716x_hybrid_pci_remove,
};

static int __devinit saa716x_hybrid_init(void)
{
	return pci_register_driver(&saa716x_hybrid_pci_driver);
}

static void __devexit saa716x_hybrid_exit(void)
{
	return pci_unregister_driver(&saa716x_hybrid_pci_driver);
}

module_init(saa716x_hybrid_init);
module_exit(saa716x_hybrid_exit);

MODULE_DESCRIPTION("SAA716x Hybrid driver");
MODULE_AUTHOR("Manu Abraham");
MODULE_LICENSE("GPL");
