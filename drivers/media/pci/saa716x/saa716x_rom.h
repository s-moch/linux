#ifndef __SAA716x_ROM_H
#define __SAA716x_ROM_H


#define MSB(__x)	((__x >> 8) & 0xff)
#define LSB(__x)	(__x & 0xff)

struct saa716x_romhdr {
	u16	header_size;
	u8	compression;
	u8	version;
	u16	data_size;
	u8	devices;
	u8	checksum;
};

struct saa716x_devinfo {
	u8	struct_size;
	u8	device_id;
	u8	master_devid;
	u8	master_busid;
	u32	device_type;
	u8	implem_id;
	u8	path_id;
	u8	gpio_id;
	u16	addr_size;
	u16	extd_data_size;
};

extern int saa716x_dump_eeprom(struct saa716x_dev *saa716x);
extern int saa716x_eeprom_data(struct saa716x_dev *saa716x);

#endif /* __SAA716x_ROM_H */
