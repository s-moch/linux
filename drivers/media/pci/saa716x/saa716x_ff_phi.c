#include <linux/types.h>

#include "saa716x_mod.h"
#include "saa716x_phi_reg.h"
#include "saa716x_priv.h"

#include "saa716x_ff.h"


unsigned int phi_mode;
module_param(phi_mode, int, 0644);
MODULE_PARM_DESC(phi_mode, "phi access mode: 0 - default, slow single word accesses;"
                                           " 1 - faster phi clock;"
                                           " 2 - fastest mode, use write-combining");


/* phi config register values: chip_select mask, ready mask, strobe time, cycle time */
#define PHI_CONFIG(__cs, __ready, __strobe, __cycle) \
	((__cs) + ((__ready) << 8) + ((__strobe) << 12) +  ((__cycle) << 20))

#define PHI_0_0 (saa716x->mmio + PHI_0 + PHI_0_0_RW_0)
#define PHI_1_0 (saa716x->mmio + PHI_1 + PHI_1_0_RW_0)
#define PHI_1_1 (sti7109->mmio_uc)
#define PHI_1_2 (sti7109->mmio_uc + 0x10000)
#define PHI_1_3 (sti7109->mmio_uc + 0x20000)
#define PHI_1_4 (sti7109->mmio_wc)
#define PHI_1_5 (sti7109->mmio_wc + 0x10000)

int saa716x_ff_phi_init(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;
	struct sti7109_dev *sti7109 = saa716x->priv;
	resource_size_t phi1_start = pci_resource_start(pdev, 0) + PHI_1;
	int err;

	if (pci_resource_len(pdev, 0) < 0x80000) {
		dprintk(SAA716x_ERROR, 1, "wrong BAR0 length");
		err = -ENODEV;
		goto fail0;
	}

	/* skip first PHI window as it is already mapped */
	sti7109->mmio_uc = ioremap_nocache(phi1_start + 0x10000, 0x30000);
	if (!sti7109->mmio_uc) {
		dprintk(SAA716x_ERROR, 1, "Mem PHI1 remap failed");
		err = -ENODEV;
		goto fail0;
	}

	sti7109->mmio_wc = ioremap_wc(phi1_start + 0x40000, 0x20000);
	if (!sti7109->mmio_wc) {
		dprintk(SAA716x_ERROR, 1, "Mem PHI1 WC remap failed");
		err = -ENODEV;
		goto fail1;
	}

	/* init PHI 0 to FIFO mode */
	SAA716x_EPWR(PHI_0, PHI_0_MODE, PHI_FIFO_MODE);

	/* init PHI 1 to SRAM mode, auto increment on */
	SAA716x_EPWR(PHI_0, PHI_1_MODE, PHI_AUTO_INCREMENT);

	/* ALE is active high */
	SAA716x_EPWR(PHI_0, PHI_POLARITY, PHI_ALE_POL);
	SAA716x_EPWR(PHI_0, PHI_TIMEOUT, 0x2a);

	/* start with PHI settings that should work on all versions of the FPGA
	   firmware */
	if (phi_mode) {
		/* fast PHI clock */
		saa716x_set_clk(saa716x, CLK_DOMAIN_PHI, PLL_FREQ);

		SAA716x_EPWR(PHI_0, PHI_0_0_CONFIG, PHI_CONFIG(0x02, 0, 6, 12));
		SAA716x_EPWR(PHI_0, PHI_1_0_CONFIG, PHI_CONFIG(0x01, 0, 6, 10));
	} else {
		/* slow PHI clock */
		saa716x_set_clk(saa716x, CLK_DOMAIN_PHI, PLL_FREQ / 2);

		SAA716x_EPWR(PHI_0, PHI_0_0_CONFIG, PHI_CONFIG(0x02, 0, 3, 6));
		SAA716x_EPWR(PHI_0, PHI_1_0_CONFIG, PHI_CONFIG(0x01, 0, 3, 5));
	}

	/* for the actual access use PHI mode 0 until saa716x_ff_phi_config
	   gets called */
	sti7109->phi_mode = 0;

	return 0;

fail1:
	if (sti7109->mmio_wc)
		iounmap(sti7109->mmio_wc);
	if (sti7109->mmio_uc)
		iounmap(sti7109->mmio_uc);
fail0:
	return err;
}

void saa716x_ff_phi_exit(struct saa716x_dev *saa716x)
{
	struct sti7109_dev *sti7109 = saa716x->priv;

	if (sti7109->mmio_wc)
		iounmap(sti7109->mmio_wc);
	if (sti7109->mmio_uc)
		iounmap(sti7109->mmio_uc);
}

void saa716x_ff_phi_config(struct saa716x_dev *saa716x)
{
	struct sti7109_dev *sti7109 = saa716x->priv;

	if (sti7109->fpga_version < 0x110)
		return;

	if (phi_mode) {
		/* 8k fifo window */
		SAA716x_EPWR(PHI_0, PHI_0_0_CONFIG, PHI_CONFIG(0x02, 0, 3, 5));
		SAA716x_EPWR(PHI_0, PHI_0_1_CONFIG, PHI_CONFIG(0x02, 0, 3, 5));
		SAA716x_EPWR(PHI_0, PHI_0_2_CONFIG, PHI_CONFIG(0x02, 0, 3, 5));
		SAA716x_EPWR(PHI_0, PHI_0_3_CONFIG, PHI_CONFIG(0x02, 0, 3, 5));

		/* noncached slow read/write window, for single-word accesses */
		SAA716x_EPWR(PHI_0, PHI_1_0_CONFIG, PHI_CONFIG(0x01, 0, 6, 10));
		/* slower noncached read window */
		SAA716x_EPWR(PHI_0, PHI_1_1_CONFIG, PHI_CONFIG(0x05, 0, 3, 8));
		/* fast noncached read window */
		SAA716x_EPWR(PHI_0, PHI_1_2_CONFIG, PHI_CONFIG(0x05, 0, 4, 6));
		/* noncached write window */
		SAA716x_EPWR(PHI_0, PHI_1_3_CONFIG, PHI_CONFIG(0x05, 0, 3, 5));
		/* write-combining dpram window */
		SAA716x_EPWR(PHI_0, PHI_1_4_CONFIG, PHI_CONFIG(0x05, 0, 3, 5));
		/* write-combining fifo window */
		SAA716x_EPWR(PHI_0, PHI_1_5_CONFIG, PHI_CONFIG(0x06, 0, 3, 5));
	} else {
		/* 8k fifo window */
		SAA716x_EPWR(PHI_0, PHI_0_0_CONFIG, PHI_CONFIG(0x02, 0, 3, 6));
		SAA716x_EPWR(PHI_0, PHI_0_1_CONFIG, PHI_CONFIG(0x02, 0, 3, 6));
		SAA716x_EPWR(PHI_0, PHI_0_2_CONFIG, PHI_CONFIG(0x02, 0, 3, 6));
		SAA716x_EPWR(PHI_0, PHI_0_3_CONFIG, PHI_CONFIG(0x02, 0, 3, 6));

		/* noncached read/write windows */
		SAA716x_EPWR(PHI_0, PHI_1_0_CONFIG, PHI_CONFIG(0x01, 0, 3, 5));
		SAA716x_EPWR(PHI_0, PHI_1_1_CONFIG, PHI_CONFIG(0x01, 0, 3, 5));
		SAA716x_EPWR(PHI_0, PHI_1_2_CONFIG, PHI_CONFIG(0x01, 0, 3, 5));
		SAA716x_EPWR(PHI_0, PHI_1_3_CONFIG, PHI_CONFIG(0x05, 0, 3, 4));
		/* write-combining dpram window */
		SAA716x_EPWR(PHI_0, PHI_1_4_CONFIG, PHI_CONFIG(0x05, 0, 3, 4));
		/* write-combining fifo window */
		SAA716x_EPWR(PHI_0, PHI_1_5_CONFIG, PHI_CONFIG(0x06, 0, 3, 4));
	}

	sti7109->phi_mode = phi_mode;
}

void saa716x_ff_phi_write(struct saa716x_dev *saa716x,
			  u32 address, const u8 *data, int length)
{
	struct sti7109_dev *sti7109 = saa716x->priv;
	int i;

	switch (sti7109->phi_mode) {
	case 2:
		memcpy(PHI_1_4 + address, data, (length+3) & ~3);
		break;
	case 1:
		memcpy_toio(PHI_1_3 + address, data, (length+3) & ~3);
		break;
	default:
		for (i = 0; i < length; i += 4) {
			SAA716x_EPWR(PHI_1, address, *((u32 *) &data[i]));
			address += 4;
		}
		break;
	}
}

void saa716x_ff_phi_read(struct saa716x_dev *saa716x,
			 u32 address, u8 *data, int length)
{
	struct sti7109_dev *sti7109 = saa716x->priv;
	int i;

	switch (sti7109->phi_mode) {
	case 2:
		memcpy(data, PHI_1_2 + address, (length+3) & ~3);
		break;
	case 1:
		memcpy_fromio(data, PHI_1_1 + address, (length+3) & ~3);
		break;
	default:
		for (i = 0; i < length; i += 4) {
			*((u32 *) &data[i]) = SAA716x_EPRD(PHI_1, address);
			address += 4;
		}
		break;
	}
}

void saa716x_ff_phi_write_fifo(struct saa716x_dev *saa716x,
			       const u8 *data, int length)
{
	struct sti7109_dev *sti7109 = saa716x->priv;
	int i;

	switch (sti7109->phi_mode) {
	case 2:
		/* special fifo access                                        */
		/* first write data in arbitrary order, then commit fifo data */
		memcpy(PHI_1_5, data, length);
		wmb();
		SAA716x_EPWR(PHI_1, FPGA_ADDR_FIFO_STAT, 0);
		break;
	case 1:
		iowrite32_rep(PHI_0_0, data, length/4);
		break;
	default:
		for (i = 0; i < length; i += 4) {
			SAA716x_EPWR(PHI_0, PHI_0_0_RW_0, *((u32 *) &data[i]));
		}
		break;
	}
}
