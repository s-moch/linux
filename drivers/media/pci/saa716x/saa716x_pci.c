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


#include "saa716x_priv.h"

#define DRIVER_NAME				"SAA716x Core"

static irqreturn_t saa716x_pci_irq(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

int __devinit saa716x_pci_init(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;
	int err = 0, ret = -ENODEV, i;
	u8 revision;
  
	dprintk(SAA716x_ERROR, 1, "found a %s PCI Express card", saa716x->config->model_name);

	err = pci_enable_device(pdev);
	if (err != 0) {
		ret = -ENODEV;
		dprintk(SAA716x_ERROR, 1, "ERROR: PCI enable failed (%i)", err);
		goto fail0;
	}

	pci_set_master(pdev);

	for (i = 0; i < 6; i++) {
		if (pci_resource_flags(pdev, i) & IORESOURCE_MEM) {
			dprintk(SAA716x_DEBUG, 1, "Found IOMEM on BAR%d", i);
			saa716x->addr = pci_resource_start(pdev, i);
			saa716x->len  = pci_resource_len(pdev, i);
			break;
		}
	}

	if (!request_mem_region(saa716x->addr, saa716x->len, DRIVER_NAME)) {
		dprintk(SAA716x_ERROR, 1, "Request region failed");
		ret = -ENODEV;
		goto fail1;
	}

	saa716x->mmio = ioremap(saa716x->addr, saa716x->len);
	if (!saa716x->mmio) {
		dprintk(SAA716x_ERROR, 1, "IO remap failed");
		ret = -ENODEV;
		goto fail2;
	}

	err = request_irq(pdev->irq,
			  saa716x_pci_irq,
			  IRQF_SHARED | IRQF_DISABLED,
			  DRIVER_NAME,
			  saa716x);

	if (err < 0) {
		dprintk(SAA716x_ERROR, 1, "SAA716x IRQ registration failed");
		ret = -ENODEV;
		goto fail3;
	}

	pci_read_config_byte(pdev, PCI_CLASS_REVISION, &revision);
	
	dprintk(SAA716x_ERROR, 0, "SAA716x Revision ID: 0x%02x", revision);

	saa716x->revision	= revision;

	dprintk(SAA716x_ERROR, 0, "    SAA%02x Rev %d [%04x:%04x], ",
		saa716x->pdev->device,
		revision,
		saa716x->pdev->subsystem_vendor,
		saa716x->pdev->subsystem_device);

	dprintk(SAA716x_ERROR, 0,
		"irq: %d,\n    memory: 0x%02lx, mmio: 0x%p\n",
		saa716x->pdev->irq,
		saa716x->addr,
		saa716x->mmio);

	pci_set_drvdata(pdev, saa716x);

	return 0;

fail3:
	dprintk(SAA716x_ERROR, 1, "Err: IO Unmap");
	if (saa716x->mmio)
		iounmap(saa716x->mmio);

fail2:
	dprintk(SAA716x_ERROR, 1, "Err: Release regions");
	release_mem_region(saa716x->addr, saa716x->len);

fail1:
	pci_disable_device(pdev);

fail0:
	pci_set_drvdata(pdev, NULL);
	return ret;
}
EXPORT_SYMBOL_GPL(saa716x_pci_init);

void __devexit saa716x_pci_exit(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;

	free_irq(pdev->irq, saa716x);

	dprintk(SAA716x_NOTICE, 1, "SAA%02x mem: 0x%p", saa716x->pdev->device, saa716x->mmio);
	if (saa716x->mmio)
		iounmap(saa716x->mmio);

	if (saa716x->addr)
		release_mem_region(saa716x->addr, saa716x->len);

	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}
EXPORT_SYMBOL_GPL(saa716x_pci_exit);

MODULE_DESCRIPTION("SAA716x bridge driver");
MODULE_AUTHOR("Manu Abraham");
MODULE_LICENSE("GPL");
