#include <linux/kernel.h>
#include <linux/string.h>
#include "saa716x_priv.h"
#include "saa716x_rom.h"
#include "saa716x_adap.h"

static int eeprom_read_byte(struct saa716x_dev *saa716x, u16 reg, u8 *val)
{
	struct saa716x_i2c *i2c = saa716x->i2c;
	struct i2c_adapter *adapter = &i2c[1].i2c_adapter;

	int ret;
	u8 b0[] = { MSB(reg), LSB(reg) };
	u8 b1;

	struct i2c_msg msg[] = {
		{ .addr = 0x50, .flags = 0,	   .buf = b0,  .len = sizeof (b0) },
		{ .addr = 0x50,	.flags = I2C_M_RD, .buf = &b1, .len = sizeof (b1) }
	};

	ret = i2c_transfer(adapter, msg, 2);
	if (ret != 2) {
		dprintk(SAA716x_ERROR, 1, "read error <reg=0x%02x, ret=%i>", reg, ret);
		return -EREMOTEIO;
	}
	*val = b1;

	return ret;
}

int saa716x_dump_eeprom(struct saa716x_dev *saa716x)
{
	u8 buf[256];
	int i, err = 0;

	for (i = 0; i < 256; i++) {
		err = eeprom_read_byte(saa716x, i, &buf[i]);
		if (err < 0) {
			dprintk(SAA716x_ERROR, 1, "EEPROM Read error");
			return err;
		}
		dprintk(SAA716x_NOTICE, 1, "EEPROM <0x%02x>=[%02x] (%c)", i, buf[i], (char )buf[i]);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(saa716x_dump_eeprom);

static int saa716x_eeprom_header(struct saa716x_dev *saa716x, struct saa716x_romhdr *rom_header)
{
	u8 buf[256];
	int i, err = 0;

	dprintk(SAA716x_ERROR, 1, "Reading ROM header %02x bytes",
		sizeof (struct saa716x_romhdr));

	for (i = 0; i < sizeof (struct saa716x_romhdr); i++) {
		err = eeprom_read_byte(saa716x, i, &buf[i]);
		if (err < 0) {
			dprintk(SAA716x_ERROR, 1, "EEPROM Read error");
			return err;
		}
		dprintk(SAA716x_NOTICE, 1, "EEPROM <0x%02x>=[%02x]", i, buf[i]);
	}

	memcpy(rom_header, buf, sizeof (struct saa716x_romhdr));
	if (rom_header->header_size != sizeof (struct saa716x_romhdr)) {
		dprintk(SAA716x_ERROR, 1, "ERROR: Header size mismatch! Read size=%d bytes, Expected=%d",
			sizeof (struct saa716x_romhdr),
			rom_header->header_size);

		return -1;
	}

	dprintk(SAA716x_ERROR, 1, "EEPROM: Data Size=%d bytes", rom_header->data_size);
	dprintk(SAA716x_ERROR, 1, "EEPROM: Version=%d", rom_header->version);
	dprintk(SAA716x_ERROR, 1, "EEPROM: Devices=%d", rom_header->devices);

	return 0;
}

int saa716x_eeprom_data(struct saa716x_dev *saa716x)
{
	struct saa716x_romhdr rom_header;

	u8 *rom_data;
	int i, ret = 0;

	/* Get header */
	ret = saa716x_eeprom_header(saa716x, &rom_header);
	if (ret != 0) {
		dprintk(SAA716x_ERROR, 1, "ERROR: Header Read failed <%d>", ret);
		goto err0;
	}

	/* Allocate length */
	rom_data = kzalloc(sizeof (u8) * rom_header.data_size, GFP_KERNEL);
	if (rom_data == NULL) {
		dprintk(SAA716x_ERROR, 1, "ERROR: out of memory");
		ret = -ENOMEM;
		goto err1;
	}

	/* Get data */
	for (i = 0; i < rom_header.data_size; i++) {
		ret = eeprom_read_byte(saa716x, i, &rom_data[i]);
		if (ret < 0) {
			dprintk(SAA716x_ERROR, 1, "EEPROM Read error");
			goto err0;
		}
		dprintk(SAA716x_NOTICE, 1, "EEPROM <0x%02x>=[%02x]", i, rom_data[i]);
	}

	return 0;

err1:
	kfree(rom_data);

err0:
	return ret;
}
EXPORT_SYMBOL_GPL(saa716x_eeprom_data);
