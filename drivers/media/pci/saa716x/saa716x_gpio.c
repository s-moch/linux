// SPDX-License-Identifier: GPL-2.0+

#include <linux/kernel.h>
#include <linux/spinlock.h>

#include "saa716x_mod.h"

#include "saa716x_gpio_reg.h"

#include "saa716x_gpio.h"
#include "saa716x_priv.h"

void saa716x_gpio_init(struct saa716x_dev *saa716x)
{
	spin_lock_init(&saa716x->gpio_lock);
}
EXPORT_SYMBOL_GPL(saa716x_gpio_init);

void saa716x_gpio_set_output(struct saa716x_dev *saa716x, int gpio)
{
	uint32_t value;

	value = SAA716x_EPRD(GPIO, GPIO_OEN);
	value &= ~(1 << gpio);
	SAA716x_EPWR(GPIO, GPIO_OEN, value);
}
EXPORT_SYMBOL_GPL(saa716x_gpio_set_output);

void saa716x_gpio_set_input(struct saa716x_dev *saa716x, int gpio)
{
	uint32_t value;

	value = SAA716x_EPRD(GPIO, GPIO_OEN);
	value |= 1 << gpio;
	SAA716x_EPWR(GPIO, GPIO_OEN, value);
}
EXPORT_SYMBOL_GPL(saa716x_gpio_set_input);

void saa716x_gpio_set_mode(struct saa716x_dev *saa716x, int gpio, int mode)
{
	uint32_t value;

	value = SAA716x_EPRD(GPIO, GPIO_WR_MODE);
	if (mode)
		value |= 1 << gpio;
	else
		value &= ~(1 << gpio);
	SAA716x_EPWR(GPIO, GPIO_WR_MODE, value);
}
EXPORT_SYMBOL_GPL(saa716x_gpio_set_mode);

void saa716x_gpio_write(struct saa716x_dev *saa716x, int gpio, int set)
{
	uint32_t value;
	unsigned long flags;

	spin_lock_irqsave(&saa716x->gpio_lock, flags);
	value = SAA716x_EPRD(GPIO, GPIO_WR);
	if (set)
		value |= 1 << gpio;
	else
		value &= ~(1 << gpio);
	SAA716x_EPWR(GPIO, GPIO_WR, value);
	spin_unlock_irqrestore(&saa716x->gpio_lock, flags);
}
EXPORT_SYMBOL_GPL(saa716x_gpio_write);

int saa716x_gpio_read(struct saa716x_dev *saa716x, int gpio)
{
	uint32_t value;

	value = SAA716x_EPRD(GPIO, GPIO_RD);
	if (value & (1 << gpio))
		return 1;
	return 0;
}
EXPORT_SYMBOL_GPL(saa716x_gpio_read);
