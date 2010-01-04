#include <linux/delay.h>
#include "saa716x_reg.h"
#include "saa716x_priv.h"

#define SAA716x_I2C_TXFAIL	(I2C_ERROR_IBE		| \
				 ACK_INTER_MTNA		| \
				 FAILURE_INTER_MAF)

#define SAA716x_I2C_TXBUSY	(TRANSMIT		| \
				 TRANSMIT_S_PROG)

#define SAA716x_I2C_RXBUSY	(RECEIVE		| \
				 RECEIVE_CLEAR)

static void saa716x_term_xfer(struct saa716x_i2c *i2c, u32 I2C_DEV)
{
	struct saa716x_dev *saa716x = i2c->saa716x;

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

static int saa716x_xfer_wait(struct saa716x_i2c *i2c, u32 I2C_DEV)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	u32 stat;
	int err = 0, timeout = 0;

	if (wait_event_timeout(i2c->i2c_wq,
			       i2c->int_stat & INTERRUPT_MTD,
			       msecs_to_jiffies(100) == -ERESTARTSYS)) {

		dprintk(SAA716x_ERROR, 1, "Master transaction failed");
		err = -EIO;
	}

	stat = SAA716x_RD(I2C_DEV, I2C_STATUS);
	while (! (stat & TRANSMIT_CLEAR)) {
		dprintk(SAA716x_ERROR, 1, "Waiting for TX FIFO to be empty");
		msleep(5);
		timeout++;
		if (timeout > 500) {
			dprintk(SAA716x_ERROR, 1, "TX FIFO empty timeout");
			saa716x_term_xfer(i2c, I2C_DEV);
			err = -EIO;
			break;
		}
	}
	return err;
}

static int saa716x_i2c_reinit(struct saa716x_i2c *i2c, u32 I2C_DEV)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	u32 reg;
	int err;

	/* Flush queue */
	SAA716x_WR(I2C_DEV, I2C_CONTROL, 0xcc);

	/* Disable all interrupts and clear status */
	SAA716x_WR(I2C_DEV, INT_CLR_ENABLE, 0x1fff);
	SAA716x_WR(I2C_DEV, INT_CLR_STATUS, 0x1fff);

	/* Reset I2C Core and generate a delay */
	SAA716x_WR(I2C_DEV, I2C_CONTROL, 0xc1);

	msleep(100);

	reg = SAA716x_RD(I2C_DEV, I2C_CONTROL);
	if (reg != 0xc0) {
		dprintk(SAA716x_ERROR, 1, "Core RESET failed");
		err = -EIO;
		goto exit;
	}
	
	/* I2C Rate Setup */
	switch (i2c->i2c_rate) {
	case SAA716x_I2C_RATE_400:
		dprintk(SAA716x_DEBUG, 1, "Reinit Adapter @ 400k");
		SAA716x_WR(I2C_DEV, CLOCK_DIVISOR_HIGH, 0x1a); /* 0.5 * 27MHz/400kHz */
		SAA716x_WR(I2C_DEV, CLOCK_DIVISOR_LOW,  0x21); /* 0.5 * 27MHz/400kHz */
		SAA716x_WR(I2C_DEV, SDA_HOLD, 0x19);
		break;
	case SAA716x_I2C_RATE_100:
		dprintk(SAA716x_DEBUG, 1, "Reinit Adapter @ 100k");
		SAA716x_WR(I2C_DEV, CLOCK_DIVISOR_HIGH, 0x68); /* 0.5 * 27MHz/400kHz */
		SAA716x_WR(I2C_DEV, CLOCK_DIVISOR_LOW,  0x87); /* 0.5 * 27MHz/400kHz */
		SAA716x_WR(I2C_DEV, SDA_HOLD, 0x60);
		break;
	default:
		dprintk(SAA716x_ERROR, 1, "Unknown Rate (Rate=0x%02x)", i2c->i2c_rate);
		break;
	}

	/* Disable all interrupts and clear status */
	SAA716x_WR(I2C_DEV, INT_CLR_ENABLE, 0x1fff);
	SAA716x_WR(I2C_DEV, INT_CLR_STATUS, 0x1fff);

	/* enable interrupts: transaction done, arbitration, No Ack and I2C error */
	SAA716x_WR(I2C_DEV, INT_SET_ENABLE, 0x00c7);

	/* Check interrupt enable status */
	reg = SAA716x_RD(I2C_DEV, INT_ENABLE);
	if (reg != 0xc7) {
		dprintk(SAA716x_ERROR, 1, "Interrupt enable failed, Exiting !");
		err = -EIO;
		goto exit;
	}
	return 0;
exit:
	dprintk(SAA716x_ERROR, 1, "I2C Reinit failed");
	return err;
}

static int saa716x_i2c_send(struct saa716x_i2c *i2c, u32 I2C_DEV, u8 data)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	int err = 0;
	u32 reg;

	/* Check FIFO status before TX */
	reg = SAA716x_RD(I2C_DEV, I2C_STATUS);
	if (reg & SAA716x_I2C_TXBUSY) {
		msleep(10);
		reg = SAA716x_RD(I2C_DEV, I2C_STATUS);
		if (reg & SAA716x_I2C_TXBUSY) {
			dprintk(SAA716x_ERROR, 1, "FIFO full or Blocked");

 			err = saa716x_i2c_reinit(i2c, I2C_DEV);
 			if (err < 0) {
 				dprintk(SAA716x_ERROR, 1, "Error Reinit");
 				err = -EIO;
				goto exit;
 			}
			err = -EBUSY;
			goto exit;
		}
		err = -EBUSY;
		goto exit;
	}

	/* Write to FIFO */
	SAA716x_WR(I2C_DEV, TX_FIFO, data);
	err = saa716x_xfer_wait(i2c, I2C_DEV);
	if (err < 0) {
		err = saa716x_i2c_reinit(i2c, I2C_DEV);
		if (err < 0) {
			dprintk(SAA716x_ERROR, 1, "Error Reinit");
			err = -EBUSY;
			goto exit;
		}
		err = -EIO;
		goto exit;
	}
	return err;

exit:
	dprintk(SAA716x_ERROR, 1, "I2C Send failed (Err=%d)", err);
	return err;
}

static int saa716x_i2c_recv(struct saa716x_i2c *i2c, u32 I2C_DEV, u8 *data)
{
	struct saa716x_dev *saa716x = i2c->saa716x;
	int err = 0;
	u32 reg;

	/* Check FIFO status before RX */
	reg = SAA716x_RD(I2C_DEV, I2C_STATUS);
	if (reg & SAA716x_I2C_RXBUSY) {
		msleep(10);
		reg = SAA716x_RD(I2C_DEV, I2C_STATUS);
		if (reg & SAA716x_I2C_RXBUSY) {
			dprintk(SAA716x_ERROR, 1, "FIFO empty");

			err = saa716x_i2c_reinit(i2c, I2C_DEV);
			if (err < 0) {
				dprintk(SAA716x_ERROR, 1, "Error Reinit");
				err = -EIO;
				goto exit;
			}
			err = -EBUSY;
			goto exit;
		}
		err = -EBUSY;
		goto exit;
	}

	/* Read from FIFO */
	*data = SAA716x_RD(I2C_DEV, RX_FIFO);

	return 0;
exit:
	dprintk(SAA716x_ERROR, 1, "Error Reading data, err=%d", err);
	return err;
}

static int saa716x_i2c_read(struct saa716x_i2c *i2c, const struct i2c_msg *msg)
{
	struct saa716x_dev *saa716x	= i2c->saa716x;
	u32 I2C_DEV			= i2c->i2c_dev;
	u8 rxd;
	int i, err = 0;

	dprintk(SAA716x_DEBUG, 0, "        %s: Address=[0x%02x] <R>[ ", __func__, msg->addr);

	/* Write */
	err = saa716x_i2c_send(i2c, I2C_DEV, START_BIT | msg->addr | 0x01);
	if (err < 0) {
		dprintk(SAA716x_ERROR, 1, "Transfer failed");
		err = -EIO;
		goto exit;
	}

	for (i = 0; i < msg->len; i++) {
		if (i == (msg->len - 1)) {
			err = saa716x_i2c_send(i2c, I2C_DEV, (u8)STOP_BIT);
			if (err < 0) {
				dprintk(SAA716x_ERROR, 1, "Transfer failed");
				err = -EIO;
				goto exit;
			}
			err = saa716x_i2c_recv(i2c, I2C_DEV, &rxd);
			if (err < 0) {
				dprintk(SAA716x_ERROR, 1, "Receive failed");
				err = -EIO;
				goto exit;
			}
			msg->buf[i] = rxd;
			dprintk(SAA716x_DEBUG, 0, "%02x ", msg->buf[i]);
		} else {
			err = saa716x_i2c_send(i2c, I2C_DEV, 0x00); /* Send dummy */
			if (err < 0) {
				dprintk(SAA716x_ERROR, 1, "Send failed");
				err = -EIO;
				goto exit;
			}
			err = saa716x_i2c_recv(i2c, I2C_DEV, &rxd);
			if (err < 0) {
				dprintk(SAA716x_ERROR, 1, "Receive failed");
				err = -EIO;
				goto exit;
			}
			msg->buf[i] = rxd;
			dprintk(SAA716x_DEBUG, 0, "%02x ", msg->buf[i]);
		}
		dprintk(SAA716x_DEBUG, 0, "]\n");
	}

	return 0;
exit:
	dprintk(SAA716x_ERROR, 1, "Error Reading data, err=%d", err);
	return err;
}

static int saa716x_i2c_write(struct saa716x_i2c *i2c, const struct i2c_msg *msg)
{
	struct saa716x_dev *saa716x	= i2c->saa716x;
	u32 I2C_DEV			= i2c->i2c_dev;

	int i, err = 0;

	/* Clear INT status before first byte */
	SAA716x_WR(I2C_DEV, INT_CLR_STATUS, 0x1fff);

	dprintk(SAA716x_DEBUG, 0, "        %s: Address=[0x%02x] <W>[ ", __func__, msg->addr);
	err = saa716x_i2c_send(i2c, I2C_DEV, START_BIT | msg->addr);
	if (err < 0) {
		dprintk(SAA716x_ERROR, 1, "Transfer failed");
		err = -EIO;
		goto exit;
	}

	for (i = 0; i < msg->len; i++) {
		dprintk(SAA716x_DEBUG, 0, "%02x ", msg->buf[i]);
		if (i == (msg->len - 1)) {
			err = saa716x_i2c_send(i2c, I2C_DEV, STOP_BIT | msg->buf[i]);
			if (err < 0) {
				dprintk(SAA716x_ERROR, 1, "Transfer failed");
				err = -EIO;
				goto exit;
			}
		} else {
			err = saa716x_i2c_send(i2c, I2C_DEV, msg->buf[i]);
			if (err < 0) {
				dprintk(SAA716x_ERROR, 1, "Transfer failed");
				err = -EIO;
				goto exit;
			}
		}
	}

	dprintk(SAA716x_DEBUG, 1, "Wrote Slave ADDRESS + START_BIT");

	return 0;
exit:
	dprintk(SAA716x_ERROR, 1, "Error Writing data, err=%d", err);
	return err;
}

static int saa716x_i2c_busa_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct saa716x_i2c *i2c = i2c_get_adapdata(adapter);
	int ret = 0, i;

	i2c = &i2c[0];
	mutex_lock(&i2c->i2c_lock);
	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_RD)
			ret = saa716x_i2c_read(i2c, &msgs[i]);
		else
			ret = saa716x_i2c_write(i2c, &msgs[i]);

		if (ret < 0)
			goto bail_out;
	}
	mutex_unlock(&i2c->i2c_lock);
	return num;

bail_out:
	mutex_unlock(&i2c->i2c_lock);
	return ret;
}

static int saa716x_i2c_busb_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct saa716x_i2c *i2c = i2c_get_adapdata(adapter);
	int ret = 0, i;

	i2c = &i2c[1];
	mutex_lock(&i2c->i2c_lock);
	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_RD)
			ret = saa716x_i2c_read(i2c, &msgs[i]);
		else
			ret = saa716x_i2c_write(i2c, &msgs[i]);

		if (ret < 0)
			goto bail_out;
	}
	mutex_unlock(&i2c->i2c_lock);
	return num;

bail_out:
	mutex_unlock(&i2c->i2c_lock);
	return ret;
}

void saa716x_i2cint_disable(struct saa716x_dev *saa716x)
{
	SAA716x_WR(I2C_A, INT_CLR_ENABLE, 0x1fff);
	SAA716x_WR(I2C_B, INT_CLR_ENABLE, 0x1fff);
	SAA716x_WR(I2C_A, INT_CLR_STATUS, 0x1fff);
	SAA716x_WR(I2C_B, INT_CLR_STATUS, 0x1fff);
}
EXPORT_SYMBOL_GPL(saa716x_i2cint_disable);

static u32 saa716x_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm saa716x_algo[] = {
	{
		.master_xfer	= saa716x_i2c_busa_xfer,
		.functionality	= saa716x_i2c_func,
	}, {
		.master_xfer	= saa716x_i2c_busb_xfer,
		.functionality	= saa716x_i2c_func,
	},
};


#define I2C_HW_B_SAA716x		0x12

static struct i2c_adapter saa716x_i2c[] = {

	{
		.owner	= THIS_MODULE,
		.name	= "SAA716x I2C Core 0",
		.id	= I2C_HW_B_SAA716x,
		.class	= I2C_CLASS_TV_DIGITAL,
		.algo	= &saa716x_algo[0],
	}, {
		.owner	= THIS_MODULE,
		.name	= "SAA716x I2C Core 1",
		.id	= I2C_HW_B_SAA716x,
		.class	= I2C_CLASS_TV_DIGITAL,
		.algo	= &saa716x_algo[1],
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
		init_waitqueue_head(&i2c->i2c_wq);

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
			err = -EIO;
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
			err = -EIO;
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

		/* Enabled interrupts:
		 * Master Transaction Done (),
		 * Master Arbitration Failure,
		 * Master Transaction No Ack,
		 * I2C Error IBE
		 * Master Transaction Data Request
		 * (0xc7)
		 */
		SAA716x_WR(I2C_DEV[i], INT_SET_ENABLE, MASTER_INTERRUPT_MTDR	| \
						       I2C_ERROR_IBE		| \
						       ENABLE_MTNA		| \
						       ENABLE_MAF		| \
						       ENABLE_MTD);

		/* Check interrupt enable status */
		reg = SAA716x_RD(I2C_DEV[i], INT_ENABLE);
		if (reg != 0xc7) {
			dprintk(SAA716x_ERROR, 1, "Adapter (%d) %s Interrupt enable failed, Exiting !", i, saa716x_i2c[i].name);
			err = -EIO;
			goto exit;
		}

		/* Check status */
		reg = SAA716x_RD(I2C_DEV[i], I2C_STATUS);
		if (!(reg & 0xd)) {
			dprintk(SAA716x_ERROR, 1, "Adapter (%d) %s has bad state, Exiting !", i, saa716x_i2c[i].name);
			err = -EIO;
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
