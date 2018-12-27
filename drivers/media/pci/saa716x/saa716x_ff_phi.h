/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __SAA716x_FF_PHI_H
#define __SAA716x_FF_PHI_H

extern int saa716x_ff_phi_init(struct saa716x_ff_dev *saa716x_ff);
extern void saa716x_ff_phi_exit(struct saa716x_ff_dev *saa716x_ff);
extern void saa716x_ff_phi_config(struct saa716x_ff_dev *saa716x_ff);
extern void saa716x_ff_phi_write(struct saa716x_ff_dev *saa716x_ff,
				 u32 address, const u8 *data, int length);
extern void saa716x_ff_phi_read(struct saa716x_ff_dev *saa716x_ff,
				u32 address, u8 *data, int length);
extern void saa716x_ff_phi_write_fifo(struct saa716x_ff_dev *saa716x_ff,
				      const u8 *data, int length);

#endif /* __SAA716x_FF_PHI_H */
