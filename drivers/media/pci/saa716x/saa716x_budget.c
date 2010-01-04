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
#include "saa716x_budget.h"

unsigned int verbose;
module_param(verbose, int, 0644);
MODULE_PARM_DESC(verbose, "verbose startup messages, default is 1 (yes)");


#define DRIVER_NAME	"SAA716x Budget"

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
	saa716x->pdev		= pdev;
	saa716x->config		= (struct saa716x_config *) pci_id->driver_data;

	err = saa716x_pci_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x PCI Initialization failed");
		goto fail1;
	}

	err = saa716x_i2c_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x I2C Initialization failed");
		goto fail2;
	}

	return 0;

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

	saa716x_i2c_exit(saa716x);
	saa716x_pci_exit(saa716x);
	kfree(saa716x);
}

static int load_config_vp1028(struct saa716x_dev *saa716x)
{
//	int ret = -ENODEV;
	int ret = 0;

	return ret;
}

#define SAA716x_MODEL_TWINHAN_VP1028	"Twinhan/Azurewave VP-1028"
#define SAA716x_DEV_TWINHAN_VP1028	"DVB-S"

static struct saa716x_config saa716x_vp1028_config = {
	.model_name		= SAA716x_MODEL_TWINHAN_VP1028,
	.dev_type		= SAA716x_DEV_TWINHAN_VP1028,
	.load_config		= &load_config_vp1028,
};


static struct pci_device_id saa716x_budget_pci_table[] = {

	MAKE_ENTRY(TWINHAN_TECHNOLOGIES, TWINHAN_VP_1028, SAA7160, &saa716x_vp1028_config) /* VP-1028 */
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
