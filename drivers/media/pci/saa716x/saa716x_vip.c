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

void saa716x_vip_disable(struct saa716x_dev *saa716x)
{
       SAA716x_WR(VI0, VIP_POWER_DOWN, VI_PWR_DWN);
       SAA716x_WR(VI1, VIP_POWER_DOWN, VI_PWR_DWN);
}
EXPORT_SYMBOL_GPL(saa716x_vip_disable);
