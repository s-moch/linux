#include <linux/kmod.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/delay.h>

#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include "saa716x_priv.h"

#define DRIVER_NAME				"SAA716x Core"

static int saa716x_request_irq(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;
	struct saa716x_config *config = saa716x->config;
	int mode, ret;

	mode = PCI_IRQ_LEGACY; /* legacy fallback mode */
	if (saa716x->int_type != MODE_INTA) {
		pci_dbg(saa716x->pdev, "Enabling MSI mode");
		mode |= PCI_IRQ_MSI;
	}

	ret = pci_alloc_irq_vectors(pdev, 1, 1, mode);
	if (ret < 0) {
		pci_err(saa716x->pdev, "IRQ vector registration failed");
		return ret;
	}
	ret = request_irq(pci_irq_vector(pdev, 0),
			  config->irq_handler,
			  IRQF_SHARED,
			  DRIVER_NAME,
			  saa716x);
	return ret;
}

static void saa716x_free_irq(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;

	free_irq(pci_irq_vector(pdev, 0), saa716x);
	pci_free_irq_vectors(pdev);
}

int saa716x_pci_init(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;
	int err = 0, ret = -ENODEV, use_dac, pm_cap;
	u32 msi_cap;
	u8 revision;

	pci_info(saa716x->pdev, "found a %s PCIe card", saa716x->config->model_name);

	err = pci_enable_device(pdev);
	if (err != 0) {
		ret = -ENODEV;
		pci_err(saa716x->pdev, "ERROR: PCI enable failed (%i)", err);
		goto fail0;
	}

	if (!pci_set_dma_mask(pdev, DMA_BIT_MASK(64))) {
		use_dac = 1;
		err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64));
		if (err) {
			pci_err(saa716x->pdev, "Unable to obtain 64bit DMA");
			goto fail1;
		}
	} else {
		err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
		if (err) {
			pci_err(saa716x->pdev, "Unable to obtain 32bit DMA");
			goto fail1;
		}
	}

	pci_set_master(pdev);

	pm_cap = pci_find_capability(pdev, PCI_CAP_ID_PM);
	if (pm_cap == 0) {
		pci_err(saa716x->pdev, "Cannot find Power Management Capability");
		err = -EIO;
		goto fail1;
	}

	if (!request_mem_region(pci_resource_start(pdev, 0),
				pci_resource_len(pdev, 0),
				DRIVER_NAME)) {

		pci_err(saa716x->pdev, "BAR0 Request failed");
		ret = -ENODEV;
		goto fail1;
	}

	if (pci_resource_len(pdev, 0) < 0x30000) {
		pci_err(saa716x->pdev, "wrong BAR0 length");
		ret = -ENODEV;
		goto fail2;
	}

	saa716x->mmio = ioremap_nocache(pci_resource_start(pdev, 0), 0x30000);
	if (!saa716x->mmio) {
		pci_err(saa716x->pdev, "Mem 0 remap failed");
		ret = -ENODEV;
		goto fail2;
	}

	err = saa716x_request_irq(saa716x);
	if (err < 0) {
		pci_err(saa716x->pdev, "IRQ registration failed, err=%d", err);
		ret = -ENODEV;
		goto fail3;
	}

	pci_read_config_byte(pdev, PCI_CLASS_REVISION, &revision);
	pci_read_config_dword(pdev, 0x40, &msi_cap);

	saa716x->revision	= revision;

	pci_info(saa716x->pdev, " SAA%x Rev %d, irq: %d%s",
		saa716x->pdev->device,
		revision,
		saa716x->pdev->irq,
		(((msi_cap >> 16) & 0x01) == 1 ? " (MSI)" : ""));

	pci_set_drvdata(pdev, saa716x);

	return 0;

fail3:
	pci_err(saa716x->pdev, "Err: IO Unmap");
	if (saa716x->mmio)
		iounmap(saa716x->mmio);
fail2:
	pci_err(saa716x->pdev, "Err: Release regions");
	release_mem_region(pci_resource_start(pdev, 0),
			   pci_resource_len(pdev, 0));

fail1:
	pci_err(saa716x->pdev, "Err: Disabling device");
	pci_disable_device(pdev);

fail0:
	pci_set_drvdata(pdev, NULL);
	return ret;
}
EXPORT_SYMBOL_GPL(saa716x_pci_init);

void saa716x_pci_exit(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;

	saa716x_free_irq(saa716x);

	if (saa716x->mmio) {
		iounmap(saa716x->mmio);
		release_mem_region(pci_resource_start(pdev, 0),
				   pci_resource_len(pdev, 0));
	}

	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}
EXPORT_SYMBOL_GPL(saa716x_pci_exit);

MODULE_DESCRIPTION("SAA716x bridge driver");
MODULE_AUTHOR("Manu Abraham");
MODULE_LICENSE("GPL");
