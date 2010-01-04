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

static irqreturn_t saa716x_pci_irq(int irq, void *dev_id)
{
	struct saa716x_dev *saa716x	= (struct saa716x_dev *) dev_id;
	struct saa716x_i2c *i2c_a	= &saa716x->i2c[0];
	struct saa716x_i2c *i2c_b	= &saa716x->i2c[1];
	u32 i2c_stat_0, i2c_stat_1, msi_stat_l, msi_stat_h;
	u32 fgpi_stat_0, fgpi_stat_1, fgpi_stat_2, fgpi_stat_3;
	u32 vi_stat_0, vi_stat_1;
	u32 dcs_stat;

	if (unlikely(saa716x == NULL)) {
		printk("%s: saa716x=NULL", __func__);
		return IRQ_NONE;
	}

	/* I2C_A/B */
	i2c_stat_0 = SAA716x_RD(I2C_A, INT_STATUS);
	i2c_stat_1 = SAA716x_RD(I2C_B, INT_STATUS);
	SAA716x_WR(I2C_A, INT_CLR_STATUS, i2c_stat_0);
	SAA716x_WR(I2C_B, INT_CLR_STATUS, i2c_stat_1);

	/* MSI */
	msi_stat_l = SAA716x_RD(MSI, MSI_INT_STATUS_L);
	msi_stat_h = SAA716x_RD(MSI, MSI_INT_STATUS_H);
	SAA716x_WR(MSI, MSI_INT_STATUS_CLR_L, msi_stat_l);
	SAA716x_WR(MSI, MSI_INT_STATUS_CLR_H, msi_stat_h);

	/* FGPI */
	fgpi_stat_0 = SAA716x_RD(FGPI0, 0xfe0);
	fgpi_stat_1 = SAA716x_RD(FGPI1, 0xfe0);
	fgpi_stat_2 = SAA716x_RD(FGPI2, 0xfe0);
	fgpi_stat_3 = SAA716x_RD(FGPI3, 0xfe0);
	SAA716x_WR(FGPI0, 0xfe8, fgpi_stat_0);
	SAA716x_WR(FGPI1, 0xfe8, fgpi_stat_1);
	SAA716x_WR(FGPI2, 0xfe8, fgpi_stat_2);
	SAA716x_WR(FGPI3, 0xfe8, fgpi_stat_3);

	/* VI0/1 */
	vi_stat_0 = SAA716x_RD(VI0, 0xfe0);
	vi_stat_1 = SAA716x_RD(VI1, 0xfe0);
	SAA716x_WR(VI0, 0xfe8, vi_stat_0);
	SAA716x_WR(VI1, 0xfe8, vi_stat_1);

	/* DCS */
	dcs_stat = SAA716x_RD(DCS, 0xfe0);
	SAA716x_WR(DCS, 0xfe8, dcs_stat);

	dprintk(SAA716x_DEBUG, 1, "SAA716x IRQ VI 0=0x%02x, 1=0x%02x", vi_stat_0, vi_stat_1);

	if (i2c_stat_0) {
		dprintk(SAA716x_DEBUG, 1, "SAA716x IRQ, I2C_A=0x%02x", i2c_stat_0);
		wake_up(&i2c_a->i2c_wq);
	}

	if (i2c_stat_1) {
		dprintk(SAA716x_DEBUG, 1, "SAA716x IRQ, I2C_B=0x%02x", i2c_stat_1);
		wake_up(&i2c_b->i2c_wq);
	}


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
