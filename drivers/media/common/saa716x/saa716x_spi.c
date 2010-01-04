#include "saa716x_priv.h"
#include "saa716x_reg.h"
#include "saa716x_spi.h"

void saa716x_spi_write(struct saa716x_dev *saa716x, const u8 *data, int length)
{
	int i;
	u32 value;
	int rounds;

	for (i = 0; i < length; i++) {
		SAA716x_EPWR(SPI, SPI_DATA, data[i]);
		rounds = 0;
		value = SAA716x_EPRD(SPI, SPI_STATUS);

		while ((value & SPI_TRANSFER_FLAG) == 0 && rounds < 5000) {
			value = SAA716x_EPRD(SPI, SPI_STATUS);
			rounds++;
		}
	}
}
EXPORT_SYMBOL_GPL(saa716x_spi_write);
