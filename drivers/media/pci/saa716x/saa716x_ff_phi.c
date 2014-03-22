#include <linux/types.h>

#include "saa716x_mod.h"
#include "saa716x_phi.h"
#include "saa716x_phi_reg.h"
#include "saa716x_priv.h"

#include "saa716x_ff.h"


/* phi config register values: chip_select mask, ready mask, strobe time, cycle time */
#define PHI_CONFIG(__cs, __ready, __strobe, __cycle) \
	((__cs) + ((__ready) << 8) + ((__strobe) << 12) +  ((__cycle) << 20))

int saa716x_ff_phi_init(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;
	struct sti7109_dev *sti7109 = saa716x->priv;
	int err;

	if (pci_resource_len(pdev, 0) < 0x80000) {
		dprintk(SAA716x_ERROR, 1, "wrong BAR0 length");
		err = -ENODEV;
		goto fail0;
	}

	sti7109->mmio_wc = ioremap_wc(pci_resource_start(pdev, 0) + 0x60000,
				      0x20000);
	if (!sti7109->mmio_wc) {
		dprintk(SAA716x_ERROR, 1, "Mem remap failed");
		err = -ENODEV;
		goto fail0;
	}

	err = saa716x_phi_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x PHI Initialization failed");
		goto fail1;
	}

	/* init PHI 0 to FIFO mode */
	SAA716x_EPWR(PHI_0, PHI_0_MODE, PHI_FIFO_MODE);
	SAA716x_EPWR(PHI_0, PHI_0_0_CONFIG, PHI_CONFIG(0x02, 0, 3, 6));

	/* init PHI 1 to SRAM mode, auto increment on */
	SAA716x_EPWR(PHI_0, PHI_1_MODE, PHI_AUTO_INCREMENT);
	SAA716x_EPWR(PHI_0, PHI_1_0_CONFIG, PHI_CONFIG(0x01, 0, 3, 5));

	/* ALE is active high */
	SAA716x_EPWR(PHI_0, PHI_POLARITY, PHI_ALE_POL);
	SAA716x_EPWR(PHI_0, PHI_TIMEOUT, 0x2a);

	return 0;

fail1:
	if (sti7109->mmio_wc)
		iounmap(sti7109->mmio_wc);
fail0:
	return err;
}

void saa716x_ff_phi_exit(struct saa716x_dev *saa716x)
{
	struct sti7109_dev *sti7109 = saa716x->priv;

	if (sti7109->mmio_wc)
		iounmap(sti7109->mmio_wc);
}

void saa716x_ff_phi_write(struct saa716x_dev *saa716x,
			  u32 address, const u8 * data, int length)
{
	int i;

	for (i = 0; i < length; i += 4) {
		SAA716x_EPWR(PHI_1, address, *((u32 *) &data[i]));
		address += 4;
	}
}

void saa716x_ff_phi_read(struct saa716x_dev *saa716x,
			 u32 address, u8 * data, int length)
{
	int i;

	for (i = 0; i < length; i += 4) {
		*((u32 *) &data[i]) = SAA716x_EPRD(PHI_1, address);
		address += 4;
	}
}

void saa716x_ff_phi_write_fifo(struct saa716x_dev *saa716x,
			       const u8 * data, int length)
{
	int i;

	for (i = 0; i < length; i += 4) {
		SAA716x_EPWR(PHI_0, PHI_0_0_RW_0, *((u32 *) &data[i]));
	}
}
