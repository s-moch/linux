#ifndef __SAA716x_PHI_H
#define __SAA716x_PHI_H

struct saa716x_dev;

extern int saa716x_phi_init(struct saa716x_dev *saa716x);
extern int saa716x_phi_write(struct saa716x_dev *saa716x, u32 address, const u8 *data, int length);
extern int saa716x_phi_read(struct saa716x_dev *saa716x, u32 address, u8 *data, int length);
extern int saa716x_phi_write_fifo(struct saa716x_dev *saa716x, const u8 * data, int length);

#endif /* __SAA716x_PHI_H */
