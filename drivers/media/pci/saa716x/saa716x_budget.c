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

#include <linux/i2c.h>
#include "saa716x_priv.h"
#include "saa716x_vip.h"
#include "saa716x_aip.h"
#include "saa716x_msi.h"
#include "saa716x_reg.h"
#include "saa716x_budget.h"
#include "saa716x_adap.h"

unsigned int verbose;
module_param(verbose, int, 0644);
MODULE_PARM_DESC(verbose, "verbose startup messages, default is 1 (yes)");

unsigned int int_type;
module_param(int_type, int, 0644);
MODULE_PARM_DESC(int_type, "force Interrupt Handler type: 0=INT-A, 1=MSI, 2=MSI-X. default INT-A mode");

#define DRIVER_NAME	"SAA716x Budget"

static int read_eeprom_byte(struct saa716x_dev *saa716x, u8 *data, u8 len)
{
	struct saa716x_i2c *i2c = saa716x->i2c;
	struct i2c_adapter *adapter = &i2c[0].i2c_adapter;

	int err;

	struct i2c_msg msg[] = {
		{.addr = 0x50, .flags = 0,		.buf = data, .len = 1},
		{.addr = 0x50, .flags = I2C_M_RD,	.buf = data, .len = len},
	};

	err = i2c_transfer(adapter, msg, 2);
	if (err < 0) {
		dprintk(SAA716x_ERROR, 1, "<err=%d, d0=0x%02x, d1=0x%02x>", err, data[0], data[1]);
		return err;
	}

	return 0;
}

static int read_eeprom(struct saa716x_dev *saa716x)
{
	u8 buf[32];
	int i, err = 0;

	err = read_eeprom_byte(saa716x, buf, 32);
	if (err < 0) {
		dprintk(SAA716x_ERROR, 1, "EEPROM Read error");
		return err;
	}
	dprintk(SAA716x_DEBUG, 0, "EEPROM=[");
	for (i = 0; i < 32; i++)
		dprintk(SAA716x_DEBUG, 0, " %02x", buf[i]);

	dprintk(SAA716x_DEBUG, 0, " ]\n");
	return 0;
}

static int __devinit saa716x_budget_pci_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
	struct saa716x_dev *saa716x;
	int err = 0;

	saa716x = kzalloc(sizeof (struct saa716x_dev), GFP_KERNEL);
	if (saa716x == NULL) {
		printk(KERN_ERR "saa716x_budget_pci_probe ERROR: out of memory\n");
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
		dprintk(SAA716x_ERROR, 1, "SAA716x Jetpack core initialization failed");
		goto fail1;
	}

	err = saa716x_i2c_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x I2C Initialization failed");
		goto fail3;
	}

	err = saa716x_dvb_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x DVB initialization failed");
		goto fail4;
	}

#if 0
	/* Experiments */
	read_eeprom(saa716x);
#endif
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

static void __devexit saa716x_budget_pci_remove(struct pci_dev *pdev)
{
	struct saa716x_dev *saa716x = pci_get_drvdata(pdev);

	saa716x_dvb_exit(saa716x);
	saa716x_i2c_exit(saa716x);
	saa716x_pci_exit(saa716x);
	kfree(saa716x);
}

static irqreturn_t saa716x_budget_pci_irq(int irq, void *dev_id)
{
	struct saa716x_dev *saa716x	= (struct saa716x_dev *) dev_id;
	struct saa716x_i2c *i2c_a	= &saa716x->i2c[0];
	struct saa716x_i2c *i2c_b	= &saa716x->i2c[1];

	if (unlikely(saa716x == NULL)) {
		printk("%s: saa716x=NULL", __func__);
		return IRQ_NONE;
	}

	dprintk(SAA716x_DEBUG, 1, "VI STAT 0=0x%02x 1=0x%02x, CTL 1=0x%02x 2=0x%02x",
		SAA716x_EPRD(VI0, INT_STATUS),
		SAA716x_EPRD(VI1, INT_STATUS),
		SAA716x_EPRD(VI0, INT_ENABLE),
		SAA716x_EPRD(VI1, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "FGPI STAT 0=0x%02x 1=0x%02x, CTL 1=0x%02x 2=0x%02x",
		SAA716x_EPRD(FGPI0, INT_STATUS),
		SAA716x_EPRD(FGPI1, INT_STATUS),
		SAA716x_EPRD(FGPI0, INT_ENABLE),
		SAA716x_EPRD(FGPI0, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "FGPI STAT 2=0x%02x 3=0x%02x, CTL 2=0x%02x 3=0x%02x",
		SAA716x_EPRD(FGPI2, INT_STATUS),
		SAA716x_EPRD(FGPI3, INT_STATUS),
		SAA716x_EPRD(FGPI2, INT_ENABLE),
		SAA716x_EPRD(FGPI3, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "AI STAT 0=0x%02x 1=0x%02x, CTL 0=0x%02x 1=0x%02x",
		SAA716x_EPRD(AI0, AI_STATUS),
		SAA716x_EPRD(AI1, AI_STATUS),
		SAA716x_EPRD(AI0, AI_CTL),
		SAA716x_EPRD(AI1, AI_CTL));

	dprintk(SAA716x_DEBUG, 1, "MSI STAT L=0x%02x H=0x%02x, CTL L=0x%02x H=0x%02x",
		SAA716x_EPRD(MSI, MSI_INT_STATUS_L),
		SAA716x_EPRD(MSI, MSI_INT_STATUS_H),
		SAA716x_EPRD(MSI, MSI_INT_ENA_L),
		SAA716x_EPRD(MSI, MSI_INT_ENA_H));

	dprintk(SAA716x_DEBUG, 1, "I2C STAT 0=0x%02x 1=0x%02x, CTL 0=0x%02x 1=0x%02x",
		SAA716x_EPRD(I2C_A, INT_STATUS),
		SAA716x_EPRD(I2C_B, INT_STATUS),
		SAA716x_EPRD(I2C_A, INT_CLR_STATUS),
		SAA716x_EPRD(I2C_B, INT_CLR_STATUS));

	dprintk(SAA716x_DEBUG, 1, "DCS STAT=0x%02x, CTL=0x%02x",
		SAA716x_EPRD(DCS, DCSC_INT_STATUS),
		SAA716x_EPRD(DCS, DCSC_INT_ENABLE));

	return IRQ_HANDLED;
}


static int load_config_vp1028(struct saa716x_dev *saa716x)
{
	int ret = 0;

	return ret;
}

#define SAA716x_MODEL_TWINHAN_VP1028	"Twinhan/Azurewave VP-1028"
#define SAA716x_DEV_TWINHAN_VP1028	"DVB-S"

static int saa716x_vp1028_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;

	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) SAA716x frontend Init", count);
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);

	return -ENODEV;
}

static struct saa716x_config saa716x_vp1028_config = {
	.model_name		= SAA716x_MODEL_TWINHAN_VP1028,
	.dev_type		= SAA716x_DEV_TWINHAN_VP1028,
	.boot_mode		= SAA716x_EXT_BOOT,
	.load_config		= &load_config_vp1028,
	.adapters		= 1,
	.frontend_attach	= saa716x_vp1028_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
};

static int load_config_vp6002(struct saa716x_dev *saa716x)
{
	int ret = 0;

	return ret;
}

#define SAA716x_MODEL_TWINHAN_VP6002	"Twinhan/Azurewave VP-6002"
#define SAA716x_DEV_TWINHAN_VP6002	"DVB-S"

static int saa716x_vp6002_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;

	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) SAA716x frontend Init", count);
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);

	return -ENODEV;
}

static struct saa716x_config saa716x_vp6002_config = {
	.model_name		= SAA716x_MODEL_TWINHAN_VP6002,
	.dev_type		= SAA716x_DEV_TWINHAN_VP6002,
	.boot_mode		= SAA716x_EXT_BOOT,
	.load_config		= &load_config_vp6002,
	.adapters		= 1,
	.frontend_attach	= saa716x_vp6002_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
};

static int load_config_knc1_duals2(struct saa716x_dev *saa716x)
{
	int ret = 0;

	return ret;
}

#define SAA716x_MODEL_KNC1_DUALS2	"KNC One Dual S2"
#define SAA716x_DEV_KNC1_DUALS2		"1xDVB-S + 1xDVB-S/S2"

static int saa716x_knc1_duals2_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;

	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) SAA716x frontend Init", count);
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);

	return -ENODEV;
}

static struct saa716x_config saa716x_knc1_duals2_config = {
	.model_name		= SAA716x_MODEL_KNC1_DUALS2,
	.dev_type		= SAA716x_DEV_KNC1_DUALS2,
	.boot_mode		= SAA716x_EXT_BOOT,
	.load_config		= &load_config_knc1_duals2,
	.adapters		= 2,
	.frontend_attach	= saa716x_knc1_duals2_frontend_attach,
	.irq_handler		= saa716x_budget_pci_irq,
};

static struct pci_device_id saa716x_budget_pci_table[] = {

	MAKE_ENTRY(TWINHAN_TECHNOLOGIES, TWINHAN_VP_1028, SAA7160, &saa716x_vp1028_config), /* VP-1028 */
	MAKE_ENTRY(TWINHAN_TECHNOLOGIES, TWINHAN_VP_6002, SAA7160, &saa716x_vp6002_config), /* VP-6002 */
	MAKE_ENTRY(KNC_One,		 KNC_Dual_S2,	  SAA7160, &saa716x_knc1_duals2_config),
	{
		.vendor = 0,
	}
};
MODULE_DEVICE_TABLE(pci, saa716x_budget_pci_table);

static struct pci_driver saa716x_budget_pci_driver = {
	.name			= DRIVER_NAME,
	.id_table		= saa716x_budget_pci_table,
	.probe			= saa716x_budget_pci_probe,
	.remove			= saa716x_budget_pci_remove,
};

static int __devinit saa716x_budget_init(void)
{
	return pci_register_driver(&saa716x_budget_pci_driver);
}

static void __devexit saa716x_budget_exit(void)
{
	return pci_unregister_driver(&saa716x_budget_pci_driver);
}

module_init(saa716x_budget_init);
module_exit(saa716x_budget_exit);

MODULE_DESCRIPTION("SAA716x Budget driver");
MODULE_AUTHOR("Manu Abraham");
MODULE_LICENSE("GPL");
