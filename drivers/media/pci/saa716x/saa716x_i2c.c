#include <linux/delay.h>

#include "saa716x_priv.h"


#define RX_FIFO			0x000
#define TX_FIFO			0x000
#define I2C_STATUS		0x008
#define I2C_CONTROL		0x00c
#define CLOCK_DIVISOR_HIGH	0x010
#define CLOCK_DIVISOR_LOW	0x014
#define RX_LEVEL		0x01c
#define TX_LEVEL		0x020
#define SDA_HOLD		0x028


#define MODULE_CONF		0xfd4
#define INT_CLR_ENABLE		0xfd8
#define INT_SET_ENABLE		0xfdc
#define INT_STATUS		0xfe0
#define INT_ENABLE		0xfe4
#define INT_CLR_STATUS		0xfe8
#define INT_SET_STATUS		0xfec


static int saa716x_i2c_read(struct saa716x_dev *saa716x, const struct i2c_msg *msg)
{

	return 0;
}

static int saa716x_i2c_write(struct saa716x_dev *saa716x, const struct i2c_msg*msg)
{

	return 0;
}

static int saa716x_i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct saa716x_i2c *i2c;
	struct saa716x_dev *saa716x;
	int ret = 0, i;

	i2c	= i2c_get_adapdata(adapter);
	saa716x = i2c->saa716x;

	mutex_lock(&i2c->i2c_lock);
	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_RD)
			ret = saa716x_i2c_read(saa716x, &msgs[i]);
		else
			ret = saa716x_i2c_write(saa716x, &msgs[i]);

		if (ret < 0)
			goto bail_out;
	}
	mutex_unlock(&i2c->i2c_lock);
	return num;

bail_out:
	mutex_unlock(&i2c->i2c_lock);
	return ret;
}

static u32 saa716x_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm saa716x_algo = {
	.master_xfer	= saa716x_i2c_xfer,
	.functionality	= saa716x_i2c_func,
};

#define I2C_HW_B_SAA716x		0x12

static struct i2c_adapter saa716x_i2c[] = {

	{
		.owner	= THIS_MODULE,
		.name	= "SAA716x I2C Core 0",
		.id	= I2C_HW_B_SAA716x,
		.class	= I2C_CLASS_TV_DIGITAL,
		.algo	= &saa716x_algo,
	}, {
		.owner	= THIS_MODULE,
		.name	= "SAA716x I2C Core 1",
		.id	= I2C_HW_B_SAA716x,
		.class	= I2C_CLASS_TV_DIGITAL,
		.algo	= &saa716x_algo,
	},
};

#define SAA716x_I2C_ADAPTERS	2

int __devinit saa716x_i2c_init(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev	= saa716x->pdev;
	struct saa716x_i2c *i2c = saa716x->i2c;
	int i, err = 0;

	dprintk(SAA716x_DEBUG, 1, "Initializing SAA%02x I2C Core", saa716x->pdev->device);
	for (i = 0; i < SAA716x_I2C_ADAPTERS; i++) {
		dprintk(SAA716x_DEBUG, 1, "Initializing adapter (%d) %s", i, saa716x_i2c[i].name);
		mutex_init(&i2c->i2c_lock);
		memcpy(&i2c->i2c_adapter, &saa716x_i2c[i], sizeof (struct i2c_adapter));
		i2c_set_adapdata(&i2c->i2c_adapter, saa716x_i2c);
		i2c->i2c_adapter.dev.parent = &pdev->dev;
		err = i2c_add_adapter(&i2c->i2c_adapter);
		if (err < 0) {
			dprintk(SAA716x_ERROR, 1, "Adapter (%d) %s init failed", i, saa716x_i2c[i].name);
			goto exit;
		}
		i2c->saa716x = saa716x;
		i2c++;
	}

	dprintk(SAA716x_DEBUG, 1, "SAA%02x I2C Core succesfully initialized", saa716x->pdev->device);

	return 0;
exit:
	return err;
}
EXPORT_SYMBOL_GPL(saa716x_i2c_init);

int __devexit saa716x_i2c_exit(struct saa716x_dev *saa716x)
{
	struct saa716x_i2c *i2c = saa716x->i2c;
	int i, err = 0;

	dprintk(SAA716x_DEBUG, 1, "Removing SAA%02x I2C Core", saa716x->pdev->device);
	for (i = 0; i < SAA716x_I2C_ADAPTERS; i++) {
		dprintk(SAA716x_DEBUG, 1, "Removing adapter (%d) %s", i, saa716x_i2c[i].name);
		err = i2c_del_adapter(&i2c->i2c_adapter);
		if (err < 0) {
			dprintk(SAA716x_ERROR, 1, "Adapter (%d) %s remove failed", i, saa716x_i2c[i].name);
			goto exit;
		}
		i2c++;
	}
	dprintk(SAA716x_DEBUG, 1, "SAA%02x I2C Core succesfully removed", saa716x->pdev->device);

	return 0;

exit:
	return err;
}
EXPORT_SYMBOL_GPL(saa716x_i2c_exit);
