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

static void saa716x_i2c_recover(struct saa716x_i2c *i2c)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	u32 I2C_DEV = SAA716x_I2C_BUS(i2c->i2c_dev);
	int i;

	u8 i2c_recover[23] = {  /* SCL/SDA high */
				0xc0,
				/* send 9 SCL pulses w/o ACK (SDA high) */
				0x40, 0xc0, 0x40, 0xc0,	0x40, 0xc0,
				0x40, 0xc0, 0x40, 0xc0,	0x40, 0xc0,
				0x40, 0xc0, 0x40, 0xc0,	0x40, 0xc0,
				/* create STOP condition */
				0x40, 0x00, 0x80, 0xc0 };

	/* Some slave may be in read state, force bus release */
	for (i = 0; i < sizeof(i2c_recover)/sizeof(*i2c_recover); i++) {
		SAA716x_EPWR(I2C_DEV, I2C_CONTROL, i2c_recover[i]);
		/* Wait long enough for possible clock stretching */
		usleep_range(100, 120);
	}
}

static void saa716x_i2c_hwinit(struct saa716x_i2c *i2c)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	struct i2c_adapter *adapter = &i2c->i2c_adapter;
	u32 I2C_DEV = SAA716x_I2C_BUS(i2c->i2c_dev);

	/* Reset I2C Core */
	SAA716x_EPWR(I2C_DEV, I2C_CONTROL, 0xc1);

	/* I2C Rate Setup, set clock divisor to 0.5 * 27MHz/i2c_rate */
	switch (i2c->i2c_rate) {
	case SAA716x_I2C_RATE_400:
		pci_info(saa716x->pdev, "Initializing %s @ 400kHz",
			adapter->name);
		SAA716x_EPWR(I2C_DEV, I2C_CLOCK_DIVISOR_HIGH, 34);
		SAA716x_EPWR(I2C_DEV, I2C_CLOCK_DIVISOR_LOW,  34);
		SAA716x_EPWR(I2C_DEV, I2C_SDA_HOLD,            2);
		break;

	case SAA716x_I2C_RATE_100:
		pci_info(saa716x->pdev, "Initializing %s @ 100kHz",
			adapter->name);
		SAA716x_EPWR(I2C_DEV, I2C_CLOCK_DIVISOR_HIGH, 135);
		SAA716x_EPWR(I2C_DEV, I2C_CLOCK_DIVISOR_LOW,  135);
		SAA716x_EPWR(I2C_DEV, I2C_SDA_HOLD,             7);
		break;

	default:
		pci_err(saa716x->pdev, "%s Unknown Rate (0x%02x)",
			adapter->name,
			i2c->i2c_rate);
		break;
	}

	saa716x_i2c_recover(i2c);
}

static int saa716x_i2c_bytetime_us(struct saa716x_i2c *i2c)
{
	return (i2c->i2c_rate == SAA716x_I2C_RATE_400) ? 23 : 90;
}

static int saa716x_i2c_send(struct saa716x_i2c *i2c, u32 data)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	u32 I2C_DEV = SAA716x_I2C_BUS(i2c->i2c_dev);
	int bytetime_us = saa716x_i2c_bytetime_us(i2c);
	u32 i2c_status;
	int i;

	for (i = 0; i < 20; i++) {
		i2c_status = SAA716x_EPRD(I2C_DEV, I2C_STATUS);
		if (!(i2c_status & I2C_TRANSMIT_PROG)) {
			/* Write to FIFO */
			SAA716x_EPWR(I2C_DEV, TX_FIFO, data);
			return 0;
		}
		usleep_range(4 * bytetime_us, 6 * bytetime_us);
	}
	return -ETIMEDOUT;
}

static int saa716x_i2c_wait(struct saa716x_i2c *i2c)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	u32 I2C_DEV = SAA716x_I2C_BUS(i2c->i2c_dev);
	int bytetime_us = saa716x_i2c_bytetime_us(i2c);
	u32 int_status, tx_bytes;
	int i;

	/* Wait for transfer done */
	for (i = 0; i < 20; i++) {
		tx_bytes = SAA716x_EPRD(I2C_DEV, I2C_TX_LEVEL);
		tx_bytes &= I2C_TRANSMIT_RANGE;
		if (tx_bytes != 0) {
			usleep_range(tx_bytes * bytetime_us,
					(tx_bytes + 1) * bytetime_us);
		} else {
			int_status = SAA716x_EPRD(I2C_DEV, INT_STATUS);
			if (int_status & I2C_ERROR_IBE)
				return -EIO;
			if (int_status & I2C_ACK_INTER_MTNA)
				return -ENXIO; /* NACK */
			if (int_status & I2C_INTERRUPT_MTD)
				return 0;
			usleep_range(bytetime_us/2, bytetime_us);
		}
	}
	return -ETIMEDOUT;
}

static int saa716x_i2c_recv(struct saa716x_i2c *i2c, u32 *data)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	u32 I2C_DEV = SAA716x_I2C_BUS(i2c->i2c_dev);
	int bytetime_us = saa716x_i2c_bytetime_us(i2c);
	u32 i2c_status, int_status;
	int i;

	for (i = 0; i < 50; i++) {
		i2c_status = SAA716x_EPRD(I2C_DEV, I2C_STATUS);
		if (!(i2c_status & I2C_RECEIVE_CLEAR)) {
			/* Read from FIFO */
			*data = SAA716x_EPRD(I2C_DEV, RX_FIFO);
			return 0;
		}
		int_status = SAA716x_EPRD(I2C_DEV, INT_STATUS);
		if (int_status & I2C_ERROR_IBE)
			return -EIO;
		if (int_status & I2C_ACK_INTER_MTNA) {
			/* Clear TX FIFO */
			SAA716x_EPWR(I2C_DEV, I2C_CONTROL, 0xc1);
			return -ENXIO; /* NACK */
		}
		usleep_range(bytetime_us/2, bytetime_us);
	}
	return -ETIMEDOUT;
}

static int saa716x_i2c_xfer_msg(struct saa716x_i2c *i2c, u16 addr,
				u8 *buf, u16 len, bool read, bool add_stop)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	u32 I2C_DEV = SAA716x_I2C_BUS(i2c->i2c_dev);
	int bytetime_us = saa716x_i2c_bytetime_us(i2c);
	u32 data, tx_bytes;
	int err, i;

	if (read && len > 8)
		return -ENOTSUPP; /* Do we need longer reads? */

	pci_dbg(saa716x->pdev, "I2C %d %s transfer, addr=0x%02x length=%d",
		i2c->i2c_dev, (read) ? "read" : "write", addr, len);

	/* Special quirks to improve read reliability */
	if (read) {
		/* Avoid returning of uninitialized data on read errors */
		for (i = 0; i < len; i++)
			buf[i] = 0;
		/* Some time between setting read address and read access */
		tx_bytes = SAA716x_EPRD(I2C_DEV, I2C_TX_LEVEL);
		tx_bytes &= I2C_TRANSMIT_RANGE;
		usleep_range((tx_bytes + 1) * bytetime_us + 100,
				(tx_bytes + 1) * bytetime_us + 120);
	}

	/* Write address and data */
	for (i = 0; i <= len; i++) {
		data = (i == 0) ?
			/* First write START with I2C address, */
			I2C_START_BIT | (addr << 1) | ((read) ? 1 : 0) :
			/* then write the data (dummy data for read) */
			buf[i-1];
		if (i > 0 && !read)
			pci_dbg(saa716x->pdev, "    <W %04x> 0x%02x", i, data);
		if (add_stop && i == len)
			data |= I2C_STOP_BIT;
		err = saa716x_i2c_send(i2c, data);
		if (err < 0) {
			pci_err(saa716x->pdev, "I2C data %d send failed", i);
			return err;
		}
	}
	if (!read || len == 0) {
		if (add_stop)
			return saa716x_i2c_wait(i2c);
		return 0;
	}

	/* Now read the data */
	for (i = 0; i < len; i++) {
		err = saa716x_i2c_recv(i2c, &data);
		if (err < 0)
			return err;
		pci_dbg(saa716x->pdev, "    <R %04x> 0x%02x", i, data);
		buf[i] = data;
	}
	SAA716x_EPWR(I2C_DEV, INT_CLR_STATUS, 0x1fff);
	return 0;
}

static int saa716x_i2c_xfer(struct i2c_adapter *adapter,
			    struct i2c_msg *msgs, int num)
{
	struct saa716x_i2c *i2c	= i2c_get_adapdata(adapter);
	struct saa716x_dev *saa716x = i2c->saa716x;
	u32 I2C_DEV = SAA716x_I2C_BUS(i2c->i2c_dev);
	bool read;
	int i, err;

	mutex_lock(&i2c->i2c_lock);

	SAA716x_EPWR(I2C_DEV, INT_CLR_STATUS, 0x1fff);

	for (i = 0; i < num; i++) {
		read = msgs[i].flags & I2C_M_RD;
		err = saa716x_i2c_xfer_msg(i2c,	msgs[i].addr, msgs[i].buf,
			msgs[i].len, read, i == (num - 1));
		if (err == -ENXIO) {
			pci_dbg(saa716x->pdev,
				"I2C %d %s NACK, msg %d, addr = 0x%02x, len=%d",
				i2c->i2c_dev, (read) ? "read" : "write", i,
				msgs[i].addr, msgs[i].len);
			break;
		} else if (err < 0) {
			pci_err(saa716x->pdev,
				"I2C %d %s transfer error, msg %d, addr = 0x%02x, len=%d, %pe",
				i2c->i2c_dev, (read) ? "read" : "write", i,
				msgs[i].addr, msgs[i].len, ERR_PTR(err));
			/* Reset I2C Core */
			SAA716x_EPWR(I2C_DEV, I2C_CONTROL, 0xc1);
			/* Finish running transfers */
			usleep_range(300, 400);
			saa716x_i2c_recover(i2c);
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
	struct pci_dev *pdev = saa716x->pdev;
	struct saa716x_i2c *i2c	= saa716x->i2c;
	struct i2c_adapter *adapter = NULL;

	int i, err = 0;

	pci_dbg(saa716x->pdev, "Initializing SAA%02x I2C Core",
		saa716x->pdev->device);

	for (i = 0; i < SAA716x_I2C_ADAPTERS; i++) {

		mutex_init(&i2c->i2c_lock);

		i2c->i2c_dev = i;
		i2c->i2c_rate = saa716x->config->i2c_rate;
		adapter	= &i2c->i2c_adapter;

		if (adapter != NULL) {

			i2c_set_adapdata(adapter, i2c);

			strcpy(adapter->name, SAA716x_I2C_ADAPTER(i));

			adapter->owner = saa716x->module;
			adapter->algo = &saa716x_algo;
			adapter->algo_data = NULL;
			adapter->dev.parent = &pdev->dev;

			pci_dbg(saa716x->pdev, "Initializing %s",
				adapter->name);

			err = i2c_add_adapter(adapter);
			if (err < 0) {
				pci_err(saa716x->pdev, "%s init failed",
					adapter->name);
				return err;
			}

			i2c->saa716x = saa716x;
			saa716x_i2c_hwinit(i2c);
		}
		i2c++;
	}

	pci_dbg(saa716x->pdev, "SAA%02x I2C Core succesfully initialized",
		saa716x->pdev->device);

	return 0;
}
EXPORT_SYMBOL_GPL(saa716x_i2c_init);

void saa716x_i2c_exit(struct saa716x_dev *saa716x)
{
	struct saa716x_i2c *i2c	= saa716x->i2c;
	struct i2c_adapter *adapter = NULL;
	int i;

	pci_dbg(saa716x->pdev, "Removing SAA%02x I2C Core",
		saa716x->pdev->device);

	for (i = 0; i < SAA716x_I2C_ADAPTERS; i++) {

		adapter = &i2c->i2c_adapter;
		pci_dbg(saa716x->pdev, "Removing %s", adapter->name);

		i2c_del_adapter(adapter);
		i2c++;
	}
}
EXPORT_SYMBOL_GPL(saa716x_i2c_exit);
