#include <linux/kernel.h>

#include "saa716x_mod.h"
#include "saa716x_phi_reg.h"
#include "saa716x_phi.h"
#include "saa716x_priv.h"



/* phi config register values: chip_select mask, ready mask, strobe time, cycle time */
#define PHI_CONFIG(__cs, __ready, __strobe, __cycle) \
	((__cs) + ((__ready) << 8) + ((__strobe) << 12) +  ((__cycle) << 20))

int saa716x_init_phi(struct saa716x_dev *saa716x, u32 port, u8 slave)
{
	int i;

	/* Reset */
	SAA716x_EPWR(PHI_0, PHI_SW_RST, 0x1);

	for (i = 0; i < 20; i++) {
		msleep(1);
		if (!(SAA716x_EPRD(PHI_0, PHI_SW_RST)))
			break;
	}

	return 0;
}

int saa716x_phi_init(struct saa716x_dev *saa716x)
{

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
}
EXPORT_SYMBOL_GPL(saa716x_phi_init);

int saa716x_phi_write(struct saa716x_dev *saa716x, u32 address, const u8 * data, int length)
{
	int i;

	for (i = 0; i < length; i += 4) {
		SAA716x_EPWR(PHI_1, address, *((u32 *) &data[i]));
		address += 4;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(saa716x_phi_write);

int saa716x_phi_read(struct saa716x_dev *saa716x, u32 address, u8 * data, int length)
{
	int i;

	for (i = 0; i < length; i += 4) {
		*((u32 *) &data[i]) = SAA716x_EPRD(PHI_1, address);
		address += 4;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(saa716x_phi_read);

int saa716x_phi_write_fifo(struct saa716x_dev *saa716x, const u8 * data, int length)
{
	int i;

	for (i = 0; i < length; i += 4) {
		SAA716x_EPWR(PHI_0, PHI_0_0_RW_0, *((u32 *) &data[i]));
	}

	return 0;
}
EXPORT_SYMBOL_GPL(saa716x_phi_write_fifo);
