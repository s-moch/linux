#include <linux/delay.h>
#include "saa716x_reg.h"
#include "saa716x_priv.h"

static void saa716x_term_xfer(struct i2c_adapter *adapter)
{
	struct saa716x_i2c *i2c;
	struct saa716x_dev *saa716x;
	u32 I2C_DEV;

	i2c 	= i2c_get_adapdata(adapter);
	saa716x = i2c->saa716x;
	I2C_DEV = i2c->i2c_dev;

	SAA716x_WR(I2C_DEV, I2C_CONTROL, 0xc0); /* Start: SCL/SDA High */
	msleep(10);
	SAA716x_WR(I2C_DEV, I2C_CONTROL, 0x80);
	msleep(10);
	SAA716x_WR(I2C_DEV, I2C_CONTROL, 0x00);
	msleep(10);
	SAA716x_WR(I2C_DEV, I2C_CONTROL, 0x80);
	msleep(10);
	SAA716x_WR(I2C_DEV, I2C_CONTROL, 0xc0);

	return;
}

static int saa716x_i2c_read(struct saa716x_dev *saa716x, const struct i2c_msg *msg)
{

	return 0;
}

//static int saa716x_i2c_write(struct saa716x_dev *saa716x, const struct i2c_msg *msg)
static int saa716x_i2c_write(struct i2c_adapter *adapter, const struct i2c_msg *msg)
{
	struct saa716x_i2c *i2c;
	struct saa716x_dev *saa716x;

	u32 stat, I2C_DEV;

	i2c	= i2c_get_adapdata(adapter);
	saa716x = i2c->saa716x;

	I2C_DEV	= i2c->i2c_dev; 
	stat	= SAA716x_RD(I2C_DEV, I2C_STATUS);
	
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

	u32 I2C_DEV[2] = {I2C_B, I2C_A};
	u32 reg;

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

		i2c->i2c_dev	= I2C_DEV[i];
		i2c->i2c_rate	= saa716x->i2c_rate;
 
		msleep(100);

		reg = SAA716x_RD(I2C_DEV[i], I2C_STATUS);
		if (!(reg & 0xd)) {
			dprintk(SAA716x_ERROR, 1, "Adapter (%d) %s RESET failed, Exiting !", i, saa716x_i2c[i].name);
			err = -1;
			goto exit;
		}

		/* Flush queue */
		SAA716x_WR(I2C_DEV[i], I2C_CONTROL, 0xcc);

		/* Disable all interrupts and clear status */
		SAA716x_WR(I2C_DEV[i], INT_CLR_ENABLE, 0x1fff);
		SAA716x_WR(I2C_DEV[i], INT_CLR_STATUS, 0x1fff);

		/* Reset I2C Core and generate a delay */
		SAA716x_WR(I2C_DEV[i], I2C_CONTROL, 0xc1);

		msleep(100);

		reg = SAA716x_RD(I2C_DEV[i], I2C_CONTROL);
		if (reg != 0xc0) {
			dprintk(SAA716x_ERROR, 1, "Core RESET failed");
			err = -1;
			goto exit;
		}
		
		/* I2C Rate Setup */
		switch (i2c->i2c_rate) {
		case SAA716x_I2C_RATE_400:
			dprintk(SAA716x_DEBUG, 1, "Initializing Adapter (%d) %s @ 400k", i, saa716x_i2c[i].name);
			SAA716x_WR(I2C_DEV[i], CLOCK_DIVISOR_HIGH, 0x1a); /* 0.5 * 27MHz/400kHz */
			SAA716x_WR(I2C_DEV[i], CLOCK_DIVISOR_LOW,  0x21); /* 0.5 * 27MHz/400kHz */
			SAA716x_WR(I2C_DEV[i], SDA_HOLD, 0x19);
			break;
		case SAA716x_I2C_RATE_100:
			dprintk(SAA716x_DEBUG, 1, "Initializing Adapter (%d) %s @ 100k", i, saa716x_i2c[i].name);
			SAA716x_WR(I2C_DEV[i], CLOCK_DIVISOR_HIGH, 0x68); /* 0.5 * 27MHz/400kHz */
			SAA716x_WR(I2C_DEV[i], CLOCK_DIVISOR_LOW,  0x87); /* 0.5 * 27MHz/400kHz */
			SAA716x_WR(I2C_DEV[i], SDA_HOLD, 0x60);
			break;
		default:
			dprintk(SAA716x_ERROR, 1, "Adapter (%d) %s Unknown Rate (Rate=0x%02x)", i, saa716x_i2c[i].name, i2c->i2c_rate);
			break;
		}

		/* Disable all interrupts and clear status */
		SAA716x_WR(I2C_DEV[i], INT_CLR_ENABLE, 0x1fff);
		SAA716x_WR(I2C_DEV[i], INT_CLR_STATUS, 0x1fff);

		/* enable interrupts: transaction done, arbitration, No Ack and I2C error */
		SAA716x_WR(I2C_DEV[i], INT_SET_ENABLE, 0x00c7);

		/* Check interrupt enable status */
		reg = SAA716x_RD(I2C_DEV[i], INT_ENABLE);
		if (reg != 0xc7) {
			dprintk(SAA716x_ERROR, 1, "Adapter (%d) %s Interrupt enable failed, Exiting !", i, saa716x_i2c[i].name);
			err = -1;
			goto exit;
		}

		/* Check status */
		reg = SAA716x_RD(I2C_DEV[i], I2C_STATUS);
		if (!(reg & 0xd)) {
			dprintk(SAA716x_ERROR, 1, "Adapter (%d) %s has bad state, Exiting !", i, saa716x_i2c[i].name);
			err = -1;
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
