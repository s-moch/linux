#include "saa716x_reg.h"
#include "saa716x_priv.h"
#include "saa716x_aip.h"

int saa716x_aip_status(struct saa716x_dev *saa716x, u32 dev)
{
	return SAA716x_EPRD(dev, AI_CTL) == 0 ? 0 : -1;
}
EXPORT_SYMBOL_GPL(saa716x_aip_status);

void saa716x_aip_disable(struct saa716x_dev *saa716x)
{
	SAA716x_EPWR(AI0, AI_CTL, 0x00);
	SAA716x_EPWR(AI1, AI_CTL, 0x00);
}
EXPORT_SYMBOL_GPL(saa716x_aip_disable);
