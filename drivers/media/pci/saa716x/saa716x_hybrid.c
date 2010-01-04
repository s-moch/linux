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

#include "saa716x_priv.h"
#include "saa716x_hybrid.h"

unsigned int verbose;
module_param(verbose, int, 0644);
MODULE_PARM_DESC(verbose, "verbose startup messages, default is 1 (yes)");


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

	saa716x_core_reset(saa716x);
	pci_read_config_dword(pdev, 0x06, &sts);
	dprintk(SAA716x_ERROR, 0, "  INTx pending=%s", ((sts >> 2) & 0x01) == 1 ? "Yes": "None");

	err = saa716x_i2c_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x I2C Initialization failed");
		goto fail3;
	}

	return 0;

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

	saa716x_i2c_exit(saa716x);
	saa716x_pci_exit(saa716x);
	kfree(saa716x);
}


/*
 * Twinhan/Azurewave VP-6090
 * DVB-S Frontend: 2x MB86A16
 * DVB-T Frontend: 2x TDA10046 + TDA8275
 */
#define SAA716x_MODEL_TWINHAN_VP6090	"Twinhan/Azurewave VP-6090"
#define SAA716x_DEV_TWINHAN_VP6090	"2xDVB-S + 2xDVB-T + 2xAnalog"

static int load_config_vp6090(struct saa716x_dev *saa716x)
{
	int ret = 0;

	return ret;
}

static struct saa716x_config saa716x_vp6090_config = {
	.model_name		= SAA716x_MODEL_TWINHAN_VP6090,
	.dev_type		= SAA716x_DEV_TWINHAN_VP6090,
	.boot_mode		= SAA716x_EXT_BOOT,
	.load_config		= &load_config_vp6090,
};


/*
 * NXP Reference design (NEMO)
 * DVB-T Frontend: 1x TDA10046 + TDA8275
 * Analog Decoder: External SAA7136
 */
#define SAA716x_MODEL_NXP_NEMO		"NEMO reference board"
#define SAA716x_DEV_NXP_NEMO		"DVB-T + Analog"

static int load_config_nemo(struct saa716x_dev *saa716x)
{
    int ret = 0;
    return ret;
}

static struct saa716x_config saa716x_nemo_config = {
	.model_name		= SAA716x_MODEL_NXP_NEMO,
	.dev_type		= SAA716x_DEV_NXP_NEMO,
	.boot_mode		= SAA716x_EXT_BOOT,
	.load_config		= &load_config_nemo,
};


static struct pci_device_id saa716x_hybrid_pci_table[] = {

	MAKE_ENTRY(TWINHAN_TECHNOLOGIES, TWINHAN_VP_6090, SAA7162, &saa716x_vp6090_config),
	MAKE_ENTRY(NXP_REFERENCE_BOARD, PCI_ANY_ID, SAA7160, &saa716x_nemo_config),
	{
		.vendor = 0,
	}
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
