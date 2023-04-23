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

	saa716x = kzalloc(sizeof (struct saa716x_dev), GFP_KERNEL);
	if (saa716x == NULL) {
		printk(KERN_ERR "saa716x_hybrid_pci_probe ERROR: out of memory\n");
		err = -ENOMEM;
		goto fail0;
	}

	saa716x->verbose	= verbose;
	saa716x->pdev		= pdev;
	saa716x->config		= (struct saa716x_config *) pci_id->driver_data;

	if ((err = saa716x_pci_init(saa716x)) != 0)
		goto fail1;


	return 0;

fail1:
	kfree(saa716x);

fail0:
	return err;
}

static void __devexit saa716x_hybrid_pci_remove(struct pci_dev *pdev)
{
	struct saa716x_dev *saa716x = pci_get_drvdata(pdev);

	saa716x_pci_exit(saa716x);
	kfree(saa716x);
}

static int load_config_vp6090(struct saa716x_dev *saa716x)
{
//	int ret = -ENODEV;
	int ret = 0;

	return ret;
}

static int load_config_nemo(struct saa716x_dev *saa716x)
{
    int ret = 0;
    return ret;
}


#define SAA716x_MODEL_TWINHAN_VP6090	"Twinhan/Azurewave VP-6090"
#define SAA716x_CHIPS_TWINHAN_VP6090       "2xTDA8263 + 2xMB86A16L + 2xTDA8275A + 2xTDA10046 + SAA7162"
#define SAA716x_DEV_TWINHAN_VP6090	"2xDVB-S + 2xDVB-T + 2xAnalog"

/*****************NXP Semiconductors reference design board ******************/
#define SAA716x_MODEL_NXP_NENO     "NXP Semiconductors NEMO referrence board" 
#define SAA716x_CHIPS_NXP_NEMO      "SAA7160 + TDA8275A + TDA10046 + SAA7136"
#define SAA716x_DEV_NXP_NEMO         "DVB-T + Analog"
/*******************************END*************************************/



static struct saa716x_config saa716x_vp6090_config = {
	.model_name		= SAA716x_MODEL_TWINHAN_VP6090,
	.chips_desc            = SAA716x_CHIPS_TWINHAN_VP6090,
	.dev_type		= SAA716x_DEV_TWINHAN_VP6090,
	.load_config		= &load_config_vp6090,
};

static struct saa716x_config saa716x_nemo_config = {
       .model_name        = SAA716x_MODEL_NXP_NENO,
	.chips_desc           = SAA716x_CHIPS_NXP_NEMO,
	.dev_type             = SAA716x_DEV_NXP_NEMO,
	.load_config          = &load_config_nemo,
};


static struct pci_device_id saa716x_hybrid_pci_table[] = {

	MAKE_ENTRY(TWINHAN_TECHNOLOGIES, TWINHAN_VP_6090, SAA7160, &saa716x_vp6090_config),
	MAKE_ENTRY(PCI_ANY_ID, PCI_ANY_ID, SAA7160, &saa716x_nemo_config),
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
