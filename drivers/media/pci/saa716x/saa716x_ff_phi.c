// SPDX-License-Identifier: GPL-2.0

#include <linux/types.h>

#include "saa716x_mod.h"
#include "saa716x_phi_reg.h"
#include "saa716x_priv.h"

#include "saa716x_ff.h"

/* phi config values: chip_select mask, ready mask, strobe time, cycle time */
#define PHI_CONFIG(__cs, __ready, __strobe, __cycle) \
	((__cs) + ((__ready) << 8) + ((__strobe) << 12) +  ((__cycle) << 20))

#define PHI_0_0 (saa716x->mmio + PHI_0 + PHI_0_0_RW_0)
#define PHI_1_0 (saa716x->mmio + PHI_1 + PHI_1_0_RW_0)
#define PHI_1_1 (sti7109->mmio)
#define PHI_1_2 (sti7109->mmio + 0x10000)
#define PHI_1_3 (sti7109->mmio + 0x20000)

int saa716x_ff_phi_init(struct saa716x_ff_dev *saa716x_ff)
{
	struct saa716x_dev *saa716x = &saa716x_ff->saa716x;
	struct sti7109_dev *sti7109 = &saa716x_ff->sti7109;
	struct pci_dev *pdev = saa716x->pdev;
	resource_size_t phi1_start = pci_resource_start(pdev, 0) + PHI_1;

	if (pci_resource_len(pdev, 0) < 0x80000) {
		pci_err(saa716x->pdev, "wrong BAR0 length");
		return -ENODEV;
	}

	/* skip first PHI window as it is already mapped */
	sti7109->mmio = ioremap(phi1_start + 0x10000, 0x30000);
	if (!sti7109->mmio) {
		pci_err(saa716x->pdev, "Mem PHI1 remap failed");
		return -ENODEV;
	}

	/* init PHI 0 to FIFO mode */
	SAA716x_EPWR(PHI_0, PHI_0_MODE, PHI_FIFO_MODE);

	/* init PHI 1 to SRAM mode, auto increment on */
	SAA716x_EPWR(PHI_0, PHI_1_MODE, PHI_AUTO_INCREMENT);

	/* ALE is active high */
	SAA716x_EPWR(PHI_0, PHI_POLARITY, PHI_ALE_POL);
	SAA716x_EPWR(PHI_0, PHI_TIMEOUT, 0x2a);

	/* fast PHI clock */
	saa716x_set_clk(saa716x, CLK_DOMAIN_PHI, PLL_FREQ);

	/* 8k fifo window */
	SAA716x_EPWR(PHI_0, PHI_0_0_CONFIG, PHI_CONFIG(0x02, 0, 6, 12));
	/* noncached slow read/write window, for single-word accesses */
	SAA716x_EPWR(PHI_0, PHI_1_0_CONFIG, PHI_CONFIG(0x01, 0, 6, 10));
	/* slower noncached read window */
	SAA716x_EPWR(PHI_0, PHI_1_1_CONFIG, PHI_CONFIG(0x05, 0, 3, 8));
	/* fast noncached read window */
	SAA716x_EPWR(PHI_0, PHI_1_2_CONFIG, PHI_CONFIG(0x05, 0, 4, 6));
	/* noncached write window */
	SAA716x_EPWR(PHI_0, PHI_1_3_CONFIG, PHI_CONFIG(0x05, 0, 3, 5));

	return 0;
}

void saa716x_ff_phi_exit(struct saa716x_ff_dev *saa716x_ff)
{
	struct sti7109_dev *sti7109 = &saa716x_ff->sti7109;

	if (sti7109->mmio)
		iounmap(sti7109->mmio);
}

void saa716x_ff_phi_write(struct saa716x_ff_dev *saa716x_ff,
			  u32 address, const u8 *data, int length)
{
	struct saa716x_dev *saa716x = &saa716x_ff->saa716x;
	struct sti7109_dev *sti7109 = &saa716x_ff->sti7109;

	void __iomem *iobase = (sti7109->fpga_version < 0x110) ?
				PHI_1_0 : PHI_1_3;
	const u32 *src = (u32 *)data;
	u32 __iomem *dst = iobase + address;
	int len = (length + 3) / 4;

	while (len--)
		writel(*src++, dst++);
}

void saa716x_ff_phi_read(struct saa716x_ff_dev *saa716x_ff,
			 u32 address, u8 *data, int length)
{
	struct saa716x_dev *saa716x = &saa716x_ff->saa716x;
	struct sti7109_dev *sti7109 = &saa716x_ff->sti7109;

	void __iomem *iobase = (sti7109->fpga_version < 0x110) ?
				PHI_1_0 : PHI_1_2;
	const u32 __iomem *src = iobase + address;
	u32 *dst = (u32 *)data;
	int len = (length + 3) / 4;

	while (len--)
		*dst++ = readl(src++);
}

void saa716x_ff_phi_write_fifo(struct saa716x_ff_dev *saa716x_ff,
			       const u8 *data, int length)
{
	struct saa716x_dev *saa716x = &saa716x_ff->saa716x;

	const u32 *src = (u32 *)data;
	u32 __iomem *dst = PHI_0_0;
	int len = length / 4;

	while (len--)
		writel(*src++, dst);
}
