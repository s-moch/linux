#include "saa716x_priv.h"
#include "saa716x_reg.h"
#include "saa716x_gpio.h"

u32 saa716x_gpio_rd(struct saa716x_dev *saa716x)
{
	return SAA716x_EPRD(GPIO, GPIO_RD);
}

void saa716x_gpio_wr(struct saa716x_dev *saa716x, u32 data)
{
	SAA716x_EPWR(GPIO, GPIO_WR, data);
}

void saa716x_gpio_ctl(struct saa716x_dev *saa716x, u32 out)
{
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&saa716x->gpio_lock, flags);

	reg  = SAA716x_EPRD(GPIO, GPIO_OEN);
	reg &= ~out;
	/* TODO ! add maskable config bits in here */
	/* reg |= (config->mask & out) */
	reg |= out;
	SAA716x_EPWR(GPIO, GPIO_OEN, reg);

	spin_unlock_irqrestore(&saa716x->gpio_lock, flags);
}

void saa716x_gpio_bits(struct saa716x_dev *saa716x, u32 bits)
{
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&saa716x->gpio_lock, flags);

	reg  = SAA716x_EPRD(GPIO, GPIO_WR);
	reg &= ~bits;
	/* TODO ! add maskable config bits in here */
	/* reg |= (config->mask & bits) */
	reg |= bits;
	SAA716x_EPWR(GPIO, GPIO_WR, reg);

	spin_unlock_irqrestore(&saa716x->gpio_lock, flags);
}

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

void saa716x_gpio_write(struct saa716x_dev *saa716x, int gpio, int set)
{
	uint32_t value;

	value = SAA716x_EPRD(GPIO, GPIO_WR);
	if (set)
		value |= 1 << gpio;
	else
		value &= ~(1 << gpio);
	SAA716x_EPWR(GPIO, GPIO_WR, value);
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
