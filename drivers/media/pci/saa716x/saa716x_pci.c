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

#include "saa716x_reg.h"
#include "saa716x_priv.h"

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
		SAA716x_RD(VI0, INT_STATUS),
		SAA716x_RD(VI1, INT_STATUS),
		SAA716x_RD(VI0, INT_ENABLE),
		SAA716x_RD(VI1, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "FGPI STAT 0=0x%02x 1=0x%02x, CTL 1=0x%02x 2=0x%02x",
		SAA716x_RD(FGPI0, INT_STATUS),
		SAA716x_RD(FGPI1, INT_STATUS),
		SAA716x_RD(FGPI0, INT_ENABLE),
		SAA716x_RD(FGPI0, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "FGPI STAT 2=0x%02x 3=0x%02x, CTL 2=0x%02x 3=0x%02x",
		SAA716x_RD(FGPI2, INT_STATUS),
		SAA716x_RD(FGPI3, INT_STATUS),
		SAA716x_RD(FGPI2, INT_ENABLE),
		SAA716x_RD(FGPI3, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "AI STAT 0=0x%02x 1=0x%02x, CTL 0=0x%02x 1=0x%02x",
		SAA716x_RD(AI0, AI_STATUS),
		SAA716x_RD(AI1, AI_STATUS),
		SAA716x_RD(AI0, AI_CTL),
		SAA716x_RD(AI1, AI_CTL));

	dprintk(SAA716x_DEBUG, 1, "MSI STAT L=0x%02x H=0x%02x, CTL L=0x%02x H=0x%02x",
		SAA716x_RD(MSI, MSI_INT_STATUS_L),
		SAA716x_RD(MSI, MSI_INT_STATUS_H),
		SAA716x_RD(MSI, MSI_INT_ENA_L),
		SAA716x_RD(MSI, MSI_INT_ENA_H));

	dprintk(SAA716x_DEBUG, 1, "I2C STAT 0=0x%02x 1=0x%02x, CTL 0=0x%02x 1=0x%02x",
		SAA716x_RD(I2C_A, INT_STATUS),
		SAA716x_RD(I2C_B, INT_STATUS),
		SAA716x_RD(I2C_A, INT_CLR_STATUS),
		SAA716x_RD(I2C_B, INT_CLR_STATUS));

	dprintk(SAA716x_DEBUG, 1, "DCS STAT=0x%02x, CTL=0x%02x",
		SAA716x_RD(DCS, DCSC_INT_STATUS),
		SAA716x_RD(DCS, DCSC_INT_ENABLE));

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
