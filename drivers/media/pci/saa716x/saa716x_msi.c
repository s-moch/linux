#include <linux/delay.h>

#include "saa716x_mod.h"
#include "saa716x_msi_reg.h"
#include "saa716x_msi.h"
#include "saa716x_priv.h"

int saa716x_msi_init(struct saa716x_dev *saa716x)
{
	int i;

	/* Reset MSI */
	SAA716x_EPWR(MSI, MSI_SW_RST, 0x1);
	for (i = 0; i < 20; i++) {
		msleep(1);
		if (!(SAA716x_EPRD(MSI, MSI_SW_RST)))
			break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(saa716x_msi_init);
