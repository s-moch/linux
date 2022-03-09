// SPDX-License-Identifier: GPL-2.0+

#include <linux/delay.h>

#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include <linux/i2c.h>

#include "saa716x_mod.h"

#include "saa716x_i2c_reg.h"
#include "saa716x_msi_reg.h"
#include "saa716x_cgu_reg.h"

#include "saa716x_i2c.h"
#include "saa716x_priv.h"

static int saa716x_i2c_hwinit(struct saa716x_i2c *i2c, u32 I2C_DEV)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	struct i2c_adapter *adapter = &i2c->i2c_adapter;

	int i, max;
	u32 reg;

	/* Reset I2C Core */
	SAA716x_EPWR(I2C_DEV, I2C_CONTROL, 0xc1);

	max = 100;
	for (i = 0; i < max; i++) {
		reg = SAA716x_EPRD(I2C_DEV, I2C_CONTROL);
		if (reg == 0xc0) {
			pci_dbg(saa716x->pdev, "Adapter (%02x) %s RESET",
				I2C_DEV, adapter->name);
			break;
		}
		usleep_range(1000, 2000);
	}
	if (i == max) {
		pci_err(saa716x->pdev, "Adapter (%02x) %s RESET failed",
			I2C_DEV, adapter->name);
		return -EIO;
	}

	/* I2C Rate Setup, set clock divisor to 0.5 * 27MHz/i2c_rate */
	switch (i2c->i2c_rate) {
	case SAA716x_I2C_RATE_400:

		pci_dbg(saa716x->pdev, "Initializing Adapter %s @ 400k",
			adapter->name);
		SAA716x_EPWR(I2C_DEV, I2C_CLOCK_DIVISOR_HIGH, 0x1a);
		SAA716x_EPWR(I2C_DEV, I2C_CLOCK_DIVISOR_LOW,  0x21);
		SAA716x_EPWR(I2C_DEV, I2C_SDA_HOLD, 0x10);
		break;

	case SAA716x_I2C_RATE_100:

		pci_dbg(saa716x->pdev, "Initializing Adapter %s @ 100k",
			adapter->name);
		SAA716x_EPWR(I2C_DEV, I2C_CLOCK_DIVISOR_HIGH, 0x68);
		SAA716x_EPWR(I2C_DEV, I2C_CLOCK_DIVISOR_LOW,  0x87);
		SAA716x_EPWR(I2C_DEV, I2C_SDA_HOLD, 0x60);
		break;

	default:

		pci_err(saa716x->pdev, "Adapter %s Unknown Rate (Rate=0x%02x)",
			adapter->name,
			i2c->i2c_rate);

		break;
	}

	/* Some slave may be in read state, force bus release */
	SAA716x_EPWR(I2C_DEV, I2C_CONTROL, 0xc0); /* SCL/SDA high */
	usleep_range(50, 200);
	for (i = 0; i < 9; i++) {                 /* Issue 9 SCL pulses w/o ACK */
		SAA716x_EPWR(I2C_DEV, I2C_CONTROL, 0x40);
		usleep_range(50, 200);
		SAA716x_EPWR(I2C_DEV, I2C_CONTROL, 0xc0);
		usleep_range(50, 200);
	}
	/* Create STOP condition */
	SAA716x_EPWR(I2C_DEV, I2C_CONTROL, 0x40); /* SCL low */
	usleep_range(50, 200);
	SAA716x_EPWR(I2C_DEV, I2C_CONTROL, 0x00); /* SCL/SDA low */
	usleep_range(50, 200);
	SAA716x_EPWR(I2C_DEV, I2C_CONTROL, 0x80); /* SDA low */
	usleep_range(50, 200);
	SAA716x_EPWR(I2C_DEV, I2C_CONTROL, 0xc0); /* SCL/SDA high */
	usleep_range(50, 200);

	return 0;
}

static int saa716x_i2c_send(struct saa716x_i2c *i2c, u32 I2C_DEV, u32 data)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	u32 reg;

	/* Check FIFO status before TX */
	reg = SAA716x_EPRD(I2C_DEV, I2C_STATUS);
	if (reg & I2C_TRANSMIT_PROG)
		return -EIO;  /* fifo full */

	/* Write to FIFO */
	SAA716x_EPWR(I2C_DEV, TX_FIFO, data);

	return 0;
}

static int saa716x_i2c_recv(struct saa716x_i2c *i2c, u32 I2C_DEV, u32 *data)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	u32 reg;
	int i;

	for (i = 0; i < 50; i++) {
		reg = SAA716x_EPRD(I2C_DEV, I2C_STATUS);
		if (!(reg & I2C_RECEIVE_CLEAR)) {
			/* Read from FIFO */
			*data = SAA716x_EPRD(I2C_DEV, RX_FIFO);
			return 0;
		}
		usleep_range(10, 20);
	}
	return -ETIMEDOUT;
}

static int saa716x_i2c_wait(struct saa716x_i2c *i2c, u32 I2C_DEV)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	int i;
	u32 i2c_status, int_status;

	/* Wait for transfer done */
	for (i = 0; i < 50; i++) {
		i2c_status = SAA716x_EPRD(I2C_DEV, I2C_STATUS);
		int_status = SAA716x_EPRD(I2C_DEV, INT_STATUS);
		if (int_status & I2C_ERROR_IBE)
			return -EIO;
		if (i2c_status & I2C_TRANSMIT_CLEAR) {
			if (int_status & I2C_ACK_INTER_MTNA)
				return -ENXIO; /* NACK */
			return 0;
		}
		usleep_range(100, 200);
	}
	return -ETIMEDOUT;
}

static int saa716x_i2c_xfer_msg(struct saa716x_i2c *i2c, u32 I2C_DEV,
				 u16 addr, u8 *buf, u16 len, u8 read, u8 add_stop)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	u32 data;
	int err;
	int i;

	SAA716x_EPWR(I2C_DEV, INT_CLR_STATUS, 0x1fff);

	/* first write START with I2C address */
	data = I2C_START_BIT | (addr << 1) | ((read) ? 1 : 0);
	pci_dbg(saa716x->pdev, "length=%d Addr:0x%02x", len, data);
	err = saa716x_i2c_send(i2c, I2C_DEV, data);
	if (err < 0) {
		pci_err(saa716x->pdev, "Address write failed");
		return err;
	}
	err = saa716x_i2c_wait(i2c, I2C_DEV);
	if (err < 0)
		return err;

	SAA716x_EPWR(I2C_DEV, INT_CLR_STATUS, 0x1fff);

	/* now write the data */
	for (i = 0; i < len; i++) {
		data = buf[i];          /* dummy data for read */
		if (read == 0)
			pci_dbg(saa716x->pdev, "    <W %04x> 0x%02x", i, data);
		if (add_stop && i == (len - 1))
			data |= I2C_STOP_BIT;
		err = saa716x_i2c_send(i2c, I2C_DEV, data);
		if (err < 0) {
			pci_err(saa716x->pdev, "Data send failed");
			return err;
		}
	}
	err = saa716x_i2c_wait(i2c, I2C_DEV);
	if (err < 0 || !read)
		return err;

	/* now read the data */
	for (i = 0; i < len; i++) {
		err = saa716x_i2c_recv(i2c, I2C_DEV, &data);
		if (err < 0) {
			pci_err(saa716x->pdev, "Data receive failed");
			return err;
		}
		pci_dbg(saa716x->pdev, "    <R %04x> 0x%02x", i, data);
		buf[i] = data;
	}
	return 0;
}

static int saa716x_i2c_xfer(struct i2c_adapter *adapter,
			    struct i2c_msg *msgs, int num)
{
	struct saa716x_i2c *i2c		= i2c_get_adapdata(adapter);
	struct saa716x_dev *saa716x	= i2c->saa716x;

	u32 DEV = SAA716x_I2C_BUS(i2c->i2c_dev);
	int i, err;

	pci_dbg(saa716x->pdev, "Bus(%02x) I2C transfer", DEV);
	mutex_lock(&i2c->i2c_lock);

	for (i = 0; i < num; i++) {
		err = saa716x_i2c_xfer_msg(i2c, DEV,
			msgs[i].addr, msgs[i].buf, msgs[i].len,
			msgs[i].flags & I2C_M_RD, i == (num - 1));
		if (err < 0) {
			if (err != -ENXIO ||
			   (msgs[i].flags == 0 && msgs[i].len != 0)) {
				pci_err(saa716x->pdev,
					"I2C transfer error, msg %d, "
					"addr = 0x%02x, len=%d, "
					"flags=0x%x, %pe",
					i, msgs[i].addr, msgs[i].len,
					msgs[i].flags, ERR_PTR(err));
				saa716x_i2c_hwinit(i2c, DEV);
			}
			break;
		}
	}

	mutex_unlock(&i2c->i2c_lock);

	if (err < 0)
		return err;
	return num;
}

static u32 saa716x_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm saa716x_algo = {
	.master_xfer	= saa716x_i2c_xfer,
	.functionality	= saa716x_i2c_func,
};

int saa716x_i2c_init(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev		= saa716x->pdev;
	struct saa716x_i2c *i2c		= saa716x->i2c;
	struct i2c_adapter *adapter	= NULL;

	int i, err = 0;

	pci_dbg(saa716x->pdev, "Initializing SAA%02x I2C Core",
		saa716x->pdev->device);

	for (i = 0; i < SAA716x_I2C_ADAPTERS; i++) {

		mutex_init(&i2c->i2c_lock);

		i2c->i2c_dev	= i;
		i2c->i2c_rate	= saa716x->config->i2c_rate;
		adapter		= &i2c->i2c_adapter;

		if (adapter != NULL) {

			i2c_set_adapdata(adapter, i2c);

			strcpy(adapter->name, SAA716x_I2C_ADAPTER(i));

			adapter->owner		= saa716x->module;
			adapter->algo		= &saa716x_algo;
			adapter->algo_data	= NULL;
			adapter->dev.parent	= &pdev->dev;

			pci_dbg(saa716x->pdev, "Initializing adapter (%d) %s",
				i,
				adapter->name);

			err = i2c_add_adapter(adapter);
			if (err < 0)
				goto exit;

			i2c->saa716x = saa716x;
			saa716x_i2c_hwinit(i2c, SAA716x_I2C_BUS(i));
		}
		i2c++;
	}

	pci_dbg(saa716x->pdev, "SAA%02x I2C Core succesfully initialized",
		saa716x->pdev->device);

	return 0;
exit:
	pci_err(saa716x->pdev, "Adapter (%d) %s init failed", i, adapter->name);
	return err;
}
EXPORT_SYMBOL_GPL(saa716x_i2c_init);

void saa716x_i2c_exit(struct saa716x_dev *saa716x)
{
	struct saa716x_i2c *i2c		= saa716x->i2c;
	struct i2c_adapter *adapter	= NULL;
	int i;

	pci_dbg(saa716x->pdev, "Removing SAA%02x I2C Core",
		saa716x->pdev->device);

	for (i = 0; i < SAA716x_I2C_ADAPTERS; i++) {

		adapter = &i2c->i2c_adapter;
		pci_dbg(saa716x->pdev, "Removing adapter (%d) %s", i,
			adapter->name);

		i2c_del_adapter(adapter);
		i2c++;
	}
}
EXPORT_SYMBOL_GPL(saa716x_i2c_exit);
