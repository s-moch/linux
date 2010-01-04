#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <linux/kmod.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include "saa716x_reg.h"
#include "saa716x_priv.h"
#include "saa716x_msi.h"

#define DRIVER_NAME				"SAA716x Core"

static const char *msi_labels[] = {
	"MSI_INT_TAGACK_VI0_0",
	"MSI_INT_TAGACK_VI0_1",
	"MSI_INT_TAGACK_VI0_2",
	"MSI_INT_TAGACK_VI1_0",
	"MSI_INT_TAGACK_VI1_1",
	"MSI_INT_TAGACK_VI1_2",
	"MSI_INT_TAGACK_FGPI_0",
	"MSI_INT_TAGACK_FGPI_1",
	"MSI_INT_TAGACK_FGPI_2",
	"MSI_INT_TAGACK_FGPI_3",
	"MSI_INT_TAGACK_AI_0",
	"MSI_INT_TAGACK_AI_1",
	"MSI_INT_OVRFLW_VI0_0",
	"MSI_INT_OVRFLW_VI0_1",
	"MSI_INT_OVRFLW_VI0_2",
	"MSI_INT_OVRFLW_VI1_0",
	"MSI_INT_OVRFLW_VI1_1",
	"MSI_INT_OVRFLW_VI1_2",
	"MSI_INT_OVRFLW_FGPI_O",
	"MSI_INT_OVRFLW_FGPI_1",
	"MSI_INT_OVRFLW_FGPI_2",
	"MSI_INT_OVRFLW_FGPI_3",
	"MSI_INT_OVRFLW_AI_0",
	"MSI_INT_OVRFLW_AI_1",
	"MSI_INT_AVINT_VI0",
	"MSI_INT_AVINT_VI1",
	"MSI_INT_AVINT_FGPI_0",
	"MSI_INT_AVINT_FGPI_1",
	"MSI_INT_AVINT_FGPI_2",
	"MSI_INT_AVINT_FGPI_3",
	"MSI_INT_AVINT_AI_0",
	"MSI_INT_AVINT_AI_1",

	"MSI_INT_UNMAPD_TC_INT",
	"MSI_INT_EXTINT_0",
	"MSI_INT_EXTINT_1",
	"MSI_INT_EXTINT_2",
	"MSI_INT_EXTINT_3",
	"MSI_INT_EXTINT_4",
	"MSI_INT_EXTINT_5",
	"MSI_INT_EXTINT_6",
	"MSI_INT_EXTINT_7",
	"MSI_INT_EXTINT_8",
	"MSI_INT_EXTINT_9",
	"MSI_INT_EXTINT_10",
	"MSI_INT_EXTINT_11",
	"MSI_INT_EXTINT_12",
	"MSI_INT_EXTINT_13",
	"MSI_INT_EXTINT_14",
	"MSI_INT_EXTINT_15",
	"MSI_INT_I2CINT_0",
	"MSI_INT_I2CINT_1"
};

static const char *i2c_labels[] = {
	"INTERRUPT_MTD"
	"FAILURE_INTER_MAF",
	"ACK_INTER_MTNA",
	"SLAVE_TRANSMIT_INTER_STSD",
	"SLAVE_RECEIVE_INTER_SRSD",
	"MODE_CHANGE_INTER_MSMC",
	"I2C_ERROR_IBE",
	"MASTER_INTERRUPT_MTDR",
	"SLAVE_INTERRUPT_STDR",
	"INTERRUPTE_RFF",
	"INTERRUPT_RFDA",
	"INTERRUPT_MTFNF",
	"INTERRUPT_STFNF",
};

struct saa716x_msix_entry {
	u8 desc[32];
	irqreturn_t (*handler)(int irq, void *dev_id);
};

static irqreturn_t saa716x_msi_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static irqreturn_t saa716x_i2c_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static struct saa716x_msix_entry saa716x_msix_handler[] = {
	{ .desc = "SAA716x_I2C_HANDLER", .handler = saa716x_i2c_handler }
};

#if 0
static irqreturn_t saa716x_pci_irq(int irq, void *dev_id)
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
#endif

static int saa716x_enable_msi(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;
	int err;

	err = pci_enable_msi(pdev);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "MSI enable failed");
		return err;
	}

	return 0;
}

static int saa716x_enable_msix(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;
	int i, ret = 0;

	for (i = 0; i < SAA716x_MSI_MAX_VECTORS; i++)
		saa716x->msix_entries[i].entry = i;

	ret = pci_enable_msix(pdev, saa716x->msix_entries, SAA716x_MSI_MAX_VECTORS);
	if (ret < 0)
		dprintk(SAA716x_ERROR, 1, "MSI-X request failed");
	if (ret > 0)
		dprintk(SAA716x_ERROR, 1, "Request exceeds available IRQ's");

	return ret;
}

static int saa716x_request_irq(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;
	struct saa716x_config *config = saa716x->config;
	int i, ret = 0;

	if (saa716x->int_type == MODE_MSI) {
		dprintk(SAA716x_DEBUG, 1, "Using MSI mode");
		ret = saa716x_enable_msi(saa716x);
	} else if (saa716x->int_type == MODE_MSI_X) {
		dprintk(SAA716x_DEBUG, 1, "Using MSI-X mode");
		ret = saa716x_enable_msix(saa716x);
	}

	if (ret) {
		dprintk(SAA716x_ERROR, 1, "INT-A Mode");
		saa716x->int_type = MODE_INTA;
	}

	if (saa716x->int_type == MODE_MSI) {
		ret = request_irq(pdev->irq,
				  saa716x_msi_handler,
				  IRQF_SHARED,
				  DRIVER_NAME,
				  saa716x);

		if (ret) {
			pci_disable_msi(pdev);
			dprintk(SAA716x_ERROR, 1, "MSI registration failed");
			ret = -EIO;
		}
	}

	if (saa716x->int_type == MODE_MSI_X) {
		for (i = 0; SAA716x_MSI_MAX_VECTORS; i++) {
			ret = request_irq(saa716x->msix_entries[i].vector,
					  saa716x_msix_handler[i].handler,
					  IRQF_SHARED,
					  saa716x_msix_handler[i].desc,
					  saa716x);

			dprintk(SAA716x_ERROR, 1, "%s @ 0x%p", saa716x_msix_handler[i].desc, saa716x_msix_handler[i].handler);
			if (ret) {
				dprintk(SAA716x_ERROR, 1, "%s MSI-X-%d registration failed", saa716x_msix_handler[i].desc, i);
				return -1;
			}
		}
	}

	if (saa716x->int_type == MODE_INTA) {
		ret = request_irq(pdev->irq,
				  config->irq_handler,
				  IRQF_SHARED,
				  DRIVER_NAME,
				  saa716x);
		if (ret < 0) {
			dprintk(SAA716x_ERROR, 1, "SAA716x IRQ registration failed");
			ret = -ENODEV;
		}
	}

	return ret;
}

static void saa716x_free_irq(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;
	int i, vector;

	if (saa716x->int_type == MODE_MSI_X) {

		for (i = 0; i < SAA716x_MSI_MAX_VECTORS; i++) {
			vector = saa716x->msix_entries[i].vector;
			free_irq(vector, saa716x);
		}

		pci_disable_msix(pdev);

	} else {
		free_irq(pdev->irq, saa716x);
		if (saa716x->int_type == MODE_MSI)
			pci_disable_msi(pdev);
	}
}

int __devinit saa716x_pci_init(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;
	int err = 0, ret = -ENODEV, i, use_dac, pm_cap;
	u32 msi_cap;
	u8 revision;

	dprintk(SAA716x_ERROR, 1, "found a %s PCIe card", saa716x->config->model_name);

	err = pci_enable_device(pdev);
	if (err != 0) {
		ret = -ENODEV;
		dprintk(SAA716x_ERROR, 1, "ERROR: PCI enable failed (%i)", err);
		goto fail0;
	}

	if (!pci_set_dma_mask(pdev, DMA_64BIT_MASK)) {
		use_dac = 1;
		err = pci_set_consistent_dma_mask(pdev, DMA_64BIT_MASK);
		if (err) {
			dprintk(SAA716x_ERROR, 1, "Unable to obtain 64bit DMA");
			goto fail1;
		}
	} else if ((err = pci_set_consistent_dma_mask(pdev, DMA_32BIT_MASK)) != 0) {
		dprintk(SAA716x_ERROR, 1, "Unable to obtain 32bit DMA");
		goto fail1;
	}

	pci_set_master(pdev);

	pm_cap = pci_find_capability(pdev, PCI_CAP_ID_PM);
	if (pm_cap == 0) {
		dprintk(SAA716x_ERROR, 1, "Cannot find Power Management Capability");
		err = -EIO;
		goto fail1;
	}

	if (!request_mem_region(pci_resource_start(pdev, 0),
				pci_resource_len(pdev, 0),
				DRIVER_NAME)) {

		dprintk(SAA716x_ERROR, 1, "BAR0 Request failed");
		ret = -ENODEV;
		goto fail1;
	}
	saa716x->mmio = ioremap(pci_resource_start(pdev, 0),
				pci_resource_len(pdev, 0));

	if (!saa716x->mmio) {
		dprintk(SAA716x_ERROR, 1, "Mem 0 remap failed");
		ret = -ENODEV;
		goto fail2;
	}

	for (i = 0; i < SAA716x_MSI_MAX_VECTORS; i++)
		saa716x->msix_entries[i].entry = i;

	err = saa716x_request_irq(saa716x);
	if (err < 0) {
		dprintk(SAA716x_ERROR, 1, "SAA716x IRQ registration failed, err=%d", err);
		ret = -ENODEV;
		goto fail3;
	}

	pci_read_config_byte(pdev, PCI_CLASS_REVISION, &revision);
	pci_read_config_dword(pdev, 0x40, &msi_cap);

	saa716x->revision	= revision;

	dprintk(SAA716x_ERROR, 0, "    SAA%02x Rev %d [%04x:%04x], ",
		saa716x->pdev->device,
		revision,
		saa716x->pdev->subsystem_vendor,
		saa716x->pdev->subsystem_device);

	dprintk(SAA716x_ERROR, 0,
		"irq: %d,\n    mmio: 0x%p\n",
		saa716x->pdev->irq,
		saa716x->mmio);

	dprintk(SAA716x_ERROR, 0, "    SAA%02x %sBit, MSI %s, MSI-X=%d msgs",
		saa716x->pdev->device,
		(((msi_cap >> 23) & 0x01) == 1 ? "64":"32"),
		(((msi_cap >> 16) & 0x01) == 1 ? "Enabled" : "Disabled"),
		(1 << ((msi_cap >> 17) & 0x07)));

	dprintk(SAA716x_ERROR, 0, "\n");

	pci_set_drvdata(pdev, saa716x);

	return 0;

fail3:
	dprintk(SAA716x_ERROR, 1, "Err: IO Unmap");
	if (saa716x->mmio)
		iounmap(saa716x->mmio);
fail2:
	dprintk(SAA716x_ERROR, 1, "Err: Release regions");
	release_mem_region(pci_resource_start(pdev, 0),
			   pci_resource_len(pdev, 0));

fail1:
	dprintk(SAA716x_ERROR, 1, "Err: Disabling device");
	pci_disable_device(pdev);

fail0:
	pci_set_drvdata(pdev, NULL);
	return ret;
}
EXPORT_SYMBOL_GPL(saa716x_pci_init);

void __devexit saa716x_pci_exit(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;

	saa716x_free_irq(saa716x);

	dprintk(SAA716x_NOTICE, 1, "SAA%02x mem0: 0x%p",
		saa716x->pdev->device,
		saa716x->mmio);

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
