#include "saa716x_reg.h"
#include "saa716x_priv.h"

void saa716x_vipint_disable(struct saa716x_dev *saa716x)
{
	SAA716x_WR(VI0, INT_ENABLE, 0); /* disable VI 0 IRQ */
	SAA716x_WR(VI1, INT_ENABLE, 0); /* disable VI 1 IRQ */
	SAA716x_WR(VI0, INT_CLR_STATUS, 0x3ff); /* clear IRQ */
	SAA716x_WR(VI1, INT_CLR_STATUS, 0x3ff); /* clear IRQ */
}
EXPORT_SYMBOL_GPL(saa716x_vipint_disable);
