#include "saa716x_reg.h"
#include "saa716x_priv.h"

void saa716x_fgpiint_disable(struct saa716x_dev *saa716x)
{
	SAA716x_EPWR(FGPI0, 0xfe4, 0); /* disable FGPI IRQ */
	SAA716x_EPWR(FGPI1, 0xfe4, 0); /* disable FGPI IRQ */
	SAA716x_EPWR(FGPI2, 0xfe4, 0); /* disable FGPI IRQ */
	SAA716x_EPWR(FGPI3, 0xfe4, 0); /* disable FGPI IRQ */
	SAA716x_EPWR(FGPI0, 0xfe8, 0x7f); /* clear status */
	SAA716x_EPWR(FGPI1, 0xfe8, 0x7f); /* clear status */
	SAA716x_EPWR(FGPI2, 0xfe8, 0x7f); /* clear status */
	SAA716x_EPWR(FGPI3, 0xfe8, 0x7f); /* clear status */
}
EXPORT_SYMBOL_GPL(saa716x_fgpiint_disable);
