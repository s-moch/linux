#ifndef __SAA716x_BOOT_H
#define __SAA716x_BOOT_H

enum saa716x_boot_mode {
	SAA716x_EXT_BOOT = 1,
	SAA716x_INT_BOOT, /* GPIO[31:30] = 0x01 */
	SAA716x_CGU_BOOT,
};

extern int saa716x_core_boot(struct saa716x_dev *saa716x);
extern int saa716x_jetpack_init(struct saa716x_dev *saa716x);

#endif /* __SAA716x_BOOT_H */
