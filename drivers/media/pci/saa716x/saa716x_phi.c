#include <linux/kernel.h>

#include "saa716x_mod.h"
#include "saa716x_phi_reg.h"
#include "saa716x_phi.h"
#include "saa716x_priv.h"


int saa716x_phi_init(struct saa716x_dev *saa716x)
{
	int i;

	/* Reset PHI */
	SAA716x_EPWR(PHI_0, PHI_SW_RST, 0x1);
	for (i = 0; i < 20; i++) {
		msleep(1);
		if (!(SAA716x_EPRD(PHI_0, PHI_SW_RST)))
			break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(saa716x_phi_init);
