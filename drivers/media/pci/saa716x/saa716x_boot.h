/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __SAA716x_BOOT_H
#define __SAA716x_BOOT_H

#define DISABLE_TIMEOUT		0x17
#define ENABLE_TIMEOUT		0x16

struct saa716x_dev;

extern int saa716x_jetpack_init(struct saa716x_dev *saa716x);

#endif /* __SAA716x_BOOT_H */
