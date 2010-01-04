#ifndef __SAA716x_GPIO_H
#define __SAA716x_GPIO_H

#define BOOT_MODE	GPIO_31 | GPIO_30
#define AV_UNIT_B	GPIO_25
#define AV_UNIT_A	GPIO_24
#define AV_INTR_B	GPIO_01
#define AV_INTR_A	GPIO_00

extern u32 saa716x_gpio_rd(struct saa716x_dev *saa716x);
extern void saa716x_gpio_wr(struct saa716x_dev *saa716x, u32 data);
extern void saa716x_gpio_ctl(struct saa716x_dev *saa716x, u32 out);
extern void saa716x_gpio_bits(struct saa716x_dev *saa716x, u32 bits);

#endif /* __SAA716x_GPIO_H */
