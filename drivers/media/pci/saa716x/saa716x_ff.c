#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/mutex.h>

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/byteorder.h>
#include <linux/kmod.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/firmware.h>

#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include <linux/i2c.h>

#include "saa716x_mod.h"

#include "saa716x_dma_reg.h"
#include "saa716x_fgpi_reg.h"
#include "saa716x_greg_reg.h"
#include "saa716x_phi_reg.h"
#include "saa716x_spi_reg.h"
#include "saa716x_msi_reg.h"

#include "saa716x_vip.h"
#include "saa716x_aip.h"
#include "saa716x_msi.h"
#include "saa716x_ff.h"
#include "saa716x_adap.h"
#include "saa716x_gpio.h"
#include "saa716x_phi.h"
#include "saa716x_rom.h"
#include "saa716x_spi.h"
#include "saa716x_priv.h"

#include <linux/dvb/video.h>
#include <linux/dvb/osd.h>

#include "stv6110x.h"
#include "stv090x.h"
#include "isl6423.h"

unsigned int verbose;
module_param(verbose, int, 0644);
MODULE_PARM_DESC(verbose, "verbose startup messages, default is 1 (yes)");

unsigned int int_type;
module_param(int_type, int, 0644);
MODULE_PARM_DESC(int_type, "force Interrupt Handler type: 0=INT-A, 1=MSI, 2=MSI-X. default INT-A mode");

#define DRIVER_NAME	"SAA716x FF"

static int saa716x_ff_fpga_init(struct saa716x_dev *saa716x)
{
	int fpgaInit;
	int fpgaDone;
	int rounds;
	int ret;
	const struct firmware *fw;

	/* request the FPGA firmware, this will block until someone uploads it */
	ret = request_firmware(&fw, "dvb-ttpremium-fpga-01.fw", &saa716x->pdev->dev);
	if (ret) {
		if (ret == -ENOENT) {
			printk(KERN_ERR "dvb-ttpremium: could not load FPGA firmware,"
			       " file not found: dvb-ttpremium-fpga-01.fw\n");
			printk(KERN_ERR "dvb-ttpremium: usually this should be in "
			       "/usr/lib/hotplug/firmware or /lib/firmware\n");
		} else
			printk(KERN_ERR "dvb-ttpremium: cannot request firmware"
			       " (error %i)\n", ret);
		return -EINVAL;
	}

	/* set FPGA PROGRAMN high */
	saa716x_gpio_write(saa716x, TT_PREMIUM_GPIO_FPGA_PROGRAMN, 1);
	msleep(10);

	/* set FPGA PROGRAMN low to set it into configuration mode */
	saa716x_gpio_write(saa716x, TT_PREMIUM_GPIO_FPGA_PROGRAMN, 0);
	msleep(10);

	/* set FPGA PROGRAMN high to start configuration process */
	saa716x_gpio_write(saa716x, TT_PREMIUM_GPIO_FPGA_PROGRAMN, 1);

	rounds = 0;
	fpgaInit = saa716x_gpio_read(saa716x, TT_PREMIUM_GPIO_FPGA_INITN);
	while (fpgaInit == 0 && rounds < 5000) {
		//msleep(1);
		fpgaInit = saa716x_gpio_read(saa716x, TT_PREMIUM_GPIO_FPGA_INITN);
		rounds++;
	}
	dprintk(SAA716x_INFO, 1, "SAA716x FPGA INITN=%d, rounds=%d", fpgaInit, rounds);

	SAA716x_EPWR(SPI, SPI_CLOCK_COUNTER, 0x08);
	SAA716x_EPWR(SPI, SPI_CONTROL_REG, SPI_MODE_SELECT);

	msleep(10);

	fpgaDone = saa716x_gpio_read(saa716x, TT_PREMIUM_GPIO_FPGA_DONE);
	dprintk(SAA716x_INFO, 1, "SAA716x FPGA DONE=%d", fpgaDone);
	dprintk(SAA716x_INFO, 1, "SAA716x FPGA write bitstream");
	saa716x_spi_write(saa716x, fw->data, fw->size);
	dprintk(SAA716x_INFO, 1, "SAA716x FPGA write bitstream done");
	fpgaDone = saa716x_gpio_read(saa716x, TT_PREMIUM_GPIO_FPGA_DONE);
	dprintk(SAA716x_INFO, 1, "SAA716x FPGA DONE=%d", fpgaDone);

	release_firmware(fw);

	if (!fpgaDone)
		return -EINVAL;

	return 0;
}

static int saa716x_ff_st7109_init(struct saa716x_dev *saa716x)
{
	int i;
	int length;
	u32 requestedBlock;
	u32 writtenBlock;
	u32 numBlocks;
	u32 blockSize;
	u32 lastBlockSize;
	u64 startTime;
	u64 currentTime;
	u64 waitTime;
	int ret;
	const struct firmware *fw;

	/* request the st7109 loader, this will block until someone uploads it */
	ret = request_firmware(&fw, "dvb-ttpremium-loader-01.fw", &saa716x->pdev->dev);
	if (ret) {
		if (ret == -ENOENT) {
			printk(KERN_ERR "dvb-ttpremium: could not load ST7109 loader,"
			       " file not found: dvb-ttpremium-loader-01.fw\n");
			printk(KERN_ERR "dvb-ttpremium: usually this should be in "
			       "/usr/lib/hotplug/firmware or /lib/firmware\n");
		} else
			printk(KERN_ERR "dvb-ttpremium: cannot request firmware"
			       " (error %i)\n", ret);
		return -EINVAL;
	}

	saa716x_phi_write(saa716x, 0, fw->data, fw->size);
	msleep(10);

	release_firmware(fw);

	/* take ST out of reset */
	saa716x_gpio_write(saa716x, TT_PREMIUM_GPIO_RESET_BACKEND, 1);

	startTime = jiffies;
	waitTime = 0;
	do {
		requestedBlock = SAA716x_EPRD(PHI_1, 0x3ffc);
		if (requestedBlock == 1)
			break;

		currentTime = jiffies;
		waitTime = currentTime - startTime;
	} while (waitTime < (1 * HZ));

	if (waitTime >= 1 * HZ) {
		dprintk(SAA716x_ERROR, 1, "STi7109 seems to be DEAD!");
		return -1;
	}
	dprintk(SAA716x_INFO, 1, "STi7109 ready after %llu ticks", waitTime);

	/* request the st7109 firmware, this will block until someone uploads it */
	ret = request_firmware(&fw, "dvb-ttpremium-st7109-01.fw", &saa716x->pdev->dev);
	if (ret) {
		if (ret == -ENOENT) {
			printk(KERN_ERR "dvb-ttpremium: could not load ST7109 firmware,"
			       " file not found: dvb-ttpremium-st7109-01.fw\n");
			printk(KERN_ERR "dvb-ttpremium: usually this should be in "
			       "/usr/lib/hotplug/firmware or /lib/firmware\n");
		} else
			printk(KERN_ERR "dvb-ttpremium: cannot request firmware"
			       " (error %i)\n", ret);
		return -EINVAL;
	}

	dprintk(SAA716x_INFO, 1, "SAA716x download ST7109 firmware");
	writtenBlock = 0;
	blockSize = 0x3c00;
	length = fw->size;
	numBlocks = length / blockSize;
	lastBlockSize = length % blockSize;
	for (i = 0; i < length; i += blockSize) {
		writtenBlock++;
		/* write one block (last may differ from blockSize) */
		if (lastBlockSize && writtenBlock == (numBlocks + 1))
			saa716x_phi_write(saa716x, 0, &fw->data[i], lastBlockSize);
		else
			saa716x_phi_write(saa716x, 0, &fw->data[i], blockSize);

		SAA716x_EPWR(PHI_1, 0x3ff8, writtenBlock);
		startTime = jiffies;
		waitTime = 0;
		do {
			requestedBlock = SAA716x_EPRD(PHI_1, 0x3ffc);
			if (requestedBlock == (writtenBlock + 1))
				break;

			currentTime = jiffies;
			waitTime = currentTime - startTime;
		} while (waitTime < (1 * HZ));

		if (waitTime >= 1 * HZ) {
			dprintk(SAA716x_ERROR, 1, "STi7109 seems to be DEAD!");
			release_firmware(fw);
			return -1;
		}
	}

	/* disable frontend support through ST firmware */
	SAA716x_EPWR(PHI_1, 0x3ff4, 1);

	/* indicate end of transfer */
	writtenBlock++;
	writtenBlock |= 0x80000000;
	SAA716x_EPWR(PHI_1, 0x3ff8, writtenBlock);

	dprintk(SAA716x_INFO, 1, "SAA716x download ST7109 firmware done");

	release_firmware(fw);

	return 0;
}

static int sti7109_raw_cmd(struct sti7109_dev * sti7109, osd_raw_cmd_t * cmd)
{
	struct saa716x_dev * saa716x = sti7109->dev;
	unsigned long timeout;

	timeout = 1 * HZ;
	timeout = wait_event_interruptible_timeout(sti7109->cmd_ready_wq,
						   sti7109->cmd_ready == 1,
						   timeout);

	if (timeout == -ERESTARTSYS || sti7109->cmd_ready == 0) {
		if (timeout == -ERESTARTSYS) {
			/* a signal arrived */
			dprintk(SAA716x_ERROR, 1, "cmd ERESTARTSYS");
			return -ERESTARTSYS;
		}
		dprintk(SAA716x_ERROR, 1, "timed out waiting for command ready");
		return -EIO;
	}

	sti7109->cmd_ready = 0;
	sti7109->result_avail = 0;
	saa716x_phi_write(saa716x, 0x0000, cmd->cmd_data, cmd->cmd_len);
	SAA716x_EPWR(PHI_1, FPGA_ADDR_PHI_ISET, ISR_CMD_MASK);

	if (cmd->result_len > 0) {
		timeout = 1 * HZ;
		timeout = wait_event_interruptible_timeout(sti7109->result_avail_wq,
							   sti7109->result_avail == 1,
							   timeout);

		if (timeout == -ERESTARTSYS || sti7109->result_avail == 0) {
			cmd->result_len = 0;
			if (timeout == -ERESTARTSYS) {
				/* a signal arrived */
				dprintk(SAA716x_ERROR, 1, "result ERESTARTSYS");
				return -ERESTARTSYS;
			}
			dprintk(SAA716x_ERROR, 1, "timed out waiting for command result");
			return -EIO;
		}

		if (sti7109->result_len > 0) {
			if (sti7109->result_len > cmd->result_len) {
				memcpy(cmd->result_data, sti7109->result_data, cmd->result_len);
			} else {
				memcpy(cmd->result_data, sti7109->result_data, sti7109->result_len);
				cmd->result_len = sti7109->result_len;
			}
		}
	}

	return 0;
}

static int sti7109_raw_data(struct sti7109_dev * sti7109, osd_raw_data_t * data)
{
	struct saa716x_dev * saa716x = sti7109->dev;
	unsigned long timeout;
	u16 blockSize;
	u16 lastBlockSize;
	u16 numBlocks;
	u16 blockIndex;
	u8 blockHeader[SIZE_BLOCK_HEADER];
	u8 * blockPtr;

	timeout = 1 * HZ;
	timeout = wait_event_interruptible_timeout(sti7109->data_ready_wq,
						   sti7109->data_ready == 1,
						   timeout);

	if (timeout == -ERESTARTSYS || sti7109->data_ready == 0) {
		if (timeout == -ERESTARTSYS) {
			/* a signal arrived */
			dprintk(SAA716x_ERROR, 1, "data ERESTARTSYS");
			return -ERESTARTSYS;
		}
		dprintk(SAA716x_ERROR, 1, "timed out waiting for data ready");
		return -EIO;
	}

	sti7109->data_ready = 0;

	/* 8 bytes is the size of the block header. Block header structure is:
	 * 16 bit - block index
	 * 16 bit - number of blocks
	 * 16 bit - current block data size
	 * 16 bit - block handle. This is used to reference the data in the command that uses it.
	 */
	blockSize = SIZE_BLOCK_DATA - SIZE_BLOCK_HEADER;
	numBlocks = data->data_length / blockSize;
	lastBlockSize = data->data_length % blockSize;
	if (lastBlockSize > 0)
		numBlocks++;

	blockHeader[2] = (u8) (numBlocks >> 8);
	blockHeader[3] = (u8) numBlocks;
	blockHeader[6] = (u8) (sti7109->data_handle >> 8);
	blockHeader[7] = (u8) sti7109->data_handle;
	blockPtr = (u8 *) data->data_buffer;
	for (blockIndex = 0; blockIndex < numBlocks; blockIndex++) {

		if (lastBlockSize && (blockIndex == (numBlocks - 1)))
			blockSize = lastBlockSize;

		blockHeader[0] = (uint8_t) (blockIndex >> 8);
		blockHeader[1] = (uint8_t) blockIndex;
		blockHeader[4] = (uint8_t) (blockSize >> 8);
		blockHeader[5] = (uint8_t) blockSize;

		sti7109->block_done = 0;
		saa716x_phi_write(saa716x, ADDR_BLOCK_DATA, blockHeader, SIZE_BLOCK_HEADER);
		saa716x_phi_write(saa716x, ADDR_BLOCK_DATA + SIZE_BLOCK_HEADER, blockPtr, blockSize);
		SAA716x_EPWR(PHI_1, FPGA_ADDR_PHI_ISET, ISR_BLOCK_MASK);

		timeout = 1 * HZ;
		timeout = wait_event_timeout(sti7109->block_done_wq,
					     sti7109->block_done == 1,
					     timeout);

		if (sti7109->block_done == 0) {
			dprintk(SAA716x_ERROR, 1, "timed out waiting for block done");
			return -EIO;
		}
		blockPtr += blockSize;
	}

	data->data_handle = sti7109->data_handle;
	sti7109->data_handle++;
	return 0;
}

static int dvb_osd_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, void *parg)
{
	struct dvb_device *dvbdev	= file->private_data;
	struct sti7109_dev *sti7109	= dvbdev->priv;
	int ret_val = -EINVAL;

	if (cmd == OSD_RAW_CMD) {
		mutex_lock(&sti7109->cmd_lock);
		ret_val = sti7109_raw_cmd(sti7109, (osd_raw_cmd_t *) parg);
		mutex_unlock(&sti7109->cmd_lock);
	}
	else if (cmd == OSD_RAW_DATA) {
		mutex_lock(&sti7109->data_lock);
		ret_val = sti7109_raw_data(sti7109, (osd_raw_data_t *) parg);
		mutex_unlock(&sti7109->data_lock);
	}

	return ret_val;
}


static struct file_operations dvb_osd_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= dvb_generic_ioctl,
	.open		= dvb_generic_open,
	.release	= dvb_generic_release,
};

static struct dvb_device dvbdev_osd = {
	.priv		= NULL,
	.users		= 1,
	.writers	= 1,
	.fops		= &dvb_osd_fops,
	.kernel_ioctl	= dvb_osd_ioctl,
};

static int saa716x_ff_osd_exit(struct saa716x_dev *saa716x)
{
	struct sti7109_dev *sti7109 = saa716x->priv;

	dvb_unregister_device(sti7109->osd_dev);
	return 0;
}

static int saa716x_ff_osd_init(struct saa716x_dev *saa716x)
{
	struct saa716x_adapter *saa716x_adap	= saa716x->saa716x_adap;
	struct sti7109_dev *sti7109		= saa716x->priv;

	dvb_register_device(&saa716x_adap->dvb_adapter,
			    &sti7109->osd_dev,
			    &dvbdev_osd,
			    sti7109,
			    DVB_DEVICE_OSD);

	return 0;
}

#define FREE_COND_TS (dvb_ringbuffer_free(&sti7109->tsout) >= TS_SIZE)

static ssize_t dvb_video_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev	= file->private_data;
	struct sti7109_dev *sti7109	= dvbdev->priv;
	struct saa716x_dev *saa716x	= sti7109->dev;
	unsigned long todo = count;

	if ((file->f_flags & O_ACCMODE) == O_RDONLY)
		return -EPERM;
/*
	if (av7110->videostate.stream_source != VIDEO_SOURCE_MEMORY)
		return -EPERM;
*/
	if ((file->f_flags & O_NONBLOCK) && !FREE_COND_TS)
		return -EWOULDBLOCK;

	while (todo >= TS_SIZE) {
		if (!FREE_COND_TS) {
			if (file->f_flags & O_NONBLOCK)
				break;
			if (wait_event_interruptible(sti7109->tsout.queue, FREE_COND_TS))
				break;
		}
		dvb_ringbuffer_write(&sti7109->tsout, buf, TS_SIZE);
		todo -= TS_SIZE;
		buf += TS_SIZE;
	}

	if (count > todo) {
		u32 fifoCtrl;

		fifoCtrl = SAA716x_EPRD(PHI_1, FPGA_ADDR_FIFO_CTRL);
		fifoCtrl |= 0x4;
		SAA716x_EPWR(PHI_1, FPGA_ADDR_FIFO_CTRL, fifoCtrl);
	}

	return count - todo;
}

static unsigned int dvb_video_poll(struct file *file, poll_table *wait)
{
	struct dvb_device *dvbdev	= file->private_data;
	struct sti7109_dev *sti7109	= dvbdev->priv;
	unsigned int mask = 0;

	if ((file->f_flags & O_ACCMODE) != O_RDONLY)
		poll_wait(file, &sti7109->tsout.queue, wait);

	if ((file->f_flags & O_ACCMODE) != O_RDONLY) {
		if (1/*sti7109->playing*/) {
			if (FREE_COND_TS)
				mask |= (POLLOUT | POLLWRNORM);
		} else /* if not playing: may play if asked for */
			mask |= (POLLOUT | POLLWRNORM);
	}

	return mask;
}

static int dvb_video_ioctl(struct inode *inode, struct file *file,
			   unsigned int cmd, void *parg)
{
	struct dvb_device *dvbdev	= file->private_data;
	struct sti7109_dev *sti7109	= dvbdev->priv;
	struct saa716x_dev *saa716x	= sti7109->dev;
	int ret = 0;

	switch (cmd) {
	case VIDEO_SELECT_SOURCE:
	{
		video_stream_source_t stream_source;

		stream_source = (video_stream_source_t) parg;
		if (stream_source == VIDEO_SOURCE_DEMUX) {
			/* select TS input 1 for TS mux 1 */
			SAA716x_EPWR(PHI_1, FPGA_ADDR_TSR_MUX1, 1);
			/* stop and reset FIFO 1 */
			SAA716x_EPWR(PHI_1, FPGA_ADDR_FIFO_CTRL, 1);
		}
		else {
			dvb_ringbuffer_flush_spinlock_wakeup(&sti7109->tsout);
			/* select FIFO 1 for TS mux 1 */
			SAA716x_EPWR(PHI_1, FPGA_ADDR_TSR_MUX1, 4);
			/* reset FIFO 1 */
			SAA716x_EPWR(PHI_1, FPGA_ADDR_FIFO_CTRL, 1);
			/* start FIFO 1 */
			SAA716x_EPWR(PHI_1, FPGA_ADDR_FIFO_CTRL, 2);
		}
		break;
	}
	case VIDEO_CLEAR_BUFFER:
	{
		dvb_ringbuffer_flush_spinlock_wakeup(&sti7109->tsout);
		break;
	}
	case VIDEO_GET_PTS:
	{
		*(u64 *)parg = sti7109->video_pts;
		break;
	}
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

static struct file_operations dvb_video_fops = {
	.owner		= THIS_MODULE,
	.write		= dvb_video_write,
	.ioctl		= dvb_generic_ioctl,
	.open		= dvb_generic_open,
	.release	= dvb_generic_release,
	.poll		= dvb_video_poll,
};

static struct dvb_device dvbdev_video = {
	.priv		= NULL,
	.users		= 1,
	.writers	= 1,
	.fops		= &dvb_video_fops,
	.kernel_ioctl	= dvb_video_ioctl,
};

static int saa716x_ff_video_exit(struct saa716x_dev *saa716x)
{
	struct sti7109_dev *sti7109 = saa716x->priv;

	dvb_unregister_device(sti7109->video_dev);
	return 0;
}

static int saa716x_ff_video_init(struct saa716x_dev *saa716x)
{
	struct saa716x_adapter *saa716x_adap	= saa716x->saa716x_adap;
	struct sti7109_dev *sti7109		= saa716x->priv;

	dvb_ringbuffer_init(&sti7109->tsout, sti7109->iobuf, TSOUT_LEN);
	sti7109->tsbuf = (u8 *) (sti7109->iobuf + TSOUT_LEN);

	dvb_register_device(&saa716x_adap->dvb_adapter,
			    &sti7109->video_dev,
			    &dvbdev_video,
			    sti7109,
			    DVB_DEVICE_VIDEO);

	return 0;
}

static int __devinit saa716x_ff_pci_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
	struct saa716x_dev *saa716x;
	struct sti7109_dev *sti7109;
	int err = 0;
	u32 value;
	unsigned long timeout;

	saa716x = kzalloc(sizeof (struct saa716x_dev), GFP_KERNEL);
	if (saa716x == NULL) {
		printk(KERN_ERR "saa716x_budget_pci_probe ERROR: out of memory\n");
		err = -ENOMEM;
		goto fail0;
	}

	saa716x->verbose	= verbose;
	saa716x->int_type	= int_type;
	saa716x->pdev		= pdev;
	saa716x->config		= (struct saa716x_config *) pci_id->driver_data;

	err = saa716x_pci_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x PCI Initialization failed");
		goto fail1;
	}

	err = saa716x_cgu_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x CGU Init failed");
		goto fail1;
	}

	err = saa716x_core_boot(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x Core Boot failed");
		goto fail2;
	}
	dprintk(SAA716x_DEBUG, 1, "SAA716x Core Boot Success");

	err = saa716x_msi_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x MSI Init failed");
		goto fail2;
	}

	err = saa716x_jetpack_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x Jetpack core initialization failed");
		goto fail1;
	}

	err = saa716x_i2c_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x I2C Initialization failed");
		goto fail3;
	}

	err = saa716x_phi_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x PHI Initialization failed");
		goto fail3;
	}

	saa716x_gpio_init(saa716x);

	/* prepare the sti7109 device struct */
	sti7109 = kzalloc(sizeof(struct sti7109_dev), GFP_KERNEL);
	if (!sti7109) {
		dprintk(SAA716x_ERROR, 1, "SAA716x: out of memory");
		goto fail3;
	}

	sti7109->dev	= saa716x;
	saa716x->priv	= sti7109;

	sti7109->iobuf = vmalloc(TSOUT_LEN + TSBUF_LEN);
	if (!sti7109->iobuf)
		goto fail4;

	mutex_init(&sti7109->cmd_lock);
	mutex_init(&sti7109->data_lock);

	init_waitqueue_head(&sti7109->boot_finish_wq);
	sti7109->boot_finished = 0;

	init_waitqueue_head(&sti7109->cmd_ready_wq);
	sti7109->cmd_ready = 0;

	init_waitqueue_head(&sti7109->result_avail_wq);
	sti7109->result_avail = 0;

	sti7109->data_handle = 0;
	init_waitqueue_head(&sti7109->data_ready_wq);
	sti7109->data_ready = 0;
	init_waitqueue_head(&sti7109->block_done_wq);
	sti7109->block_done = 0;

	saa716x_gpio_set_output(saa716x, TT_PREMIUM_GPIO_POWER_ENABLE);
	saa716x_gpio_set_output(saa716x, TT_PREMIUM_GPIO_RESET_BACKEND);
	saa716x_gpio_set_output(saa716x, TT_PREMIUM_GPIO_FPGA_CS0);
	saa716x_gpio_set_mode(saa716x, TT_PREMIUM_GPIO_FPGA_CS0, 1);
	saa716x_gpio_set_output(saa716x, TT_PREMIUM_GPIO_FPGA_CS1);
	saa716x_gpio_set_mode(saa716x, TT_PREMIUM_GPIO_FPGA_CS1, 1);
	saa716x_gpio_set_output(saa716x, TT_PREMIUM_GPIO_FPGA_PROGRAMN);
	saa716x_gpio_set_input(saa716x, TT_PREMIUM_GPIO_FPGA_DONE);
	saa716x_gpio_set_input(saa716x, TT_PREMIUM_GPIO_FPGA_INITN);

	/* hold ST in reset */
	saa716x_gpio_write(saa716x, TT_PREMIUM_GPIO_RESET_BACKEND, 0);

	/* enable board power */
	saa716x_gpio_write(saa716x, TT_PREMIUM_GPIO_POWER_ENABLE, 1);
	msleep(100);

	err = saa716x_ff_fpga_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x FF FPGA Initialization failed");
		goto fail5;
	}

	/* enable interrupts from ST7109 -> PC */
	SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICTRL, 0x3);

	value = SAA716x_EPRD(MSI, MSI_CONFIG33);
	value &= 0xFCFFFFFF;
	value |= MSI_INT_POL_EDGE_FALL;
	SAA716x_EPWR(MSI, MSI_CONFIG33, value);
	SAA716x_EPWR(MSI, MSI_INT_ENA_SET_H, MSI_INT_EXTINT_0);

	err = saa716x_ff_st7109_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x FF STi7109 initialization failed");
		goto fail5;
	}

	err = saa716x_dump_eeprom(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x EEPROM dump failed");
	}
#if 0
	err = saa716x_eeprom_data(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x EEPROM dump failed");
	}
#endif

	/* enable FGPI2 and FGPI3 for TS inputs */
	SAA716x_EPWR(GREG, GREG_VI_CTRL, 0x0689F04);
	SAA716x_EPWR(GREG, GREG_FGPI_CTRL, 0x280);

	err = saa716x_dvb_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x DVB initialization failed");
		goto fail6;
	}

	/* wait a maximum of 10 seconds for the STi7109 to boot */
	timeout = 10 * HZ;
	timeout = wait_event_interruptible_timeout(sti7109->boot_finish_wq,
						   sti7109->boot_finished == 1,
						   timeout);

	if (timeout == -ERESTARTSYS || sti7109->boot_finished == 0) {
		if (timeout == -ERESTARTSYS) {
			/* a signal arrived */
			goto fail6;
		}
		dprintk(SAA716x_ERROR, 1, "timed out waiting for boot finish");
		goto fail6;
	}
	dprintk(SAA716x_INFO, 1, "STi7109 finished booting");

	err = saa716x_ff_video_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x FF VIDEO initialization failed");
		goto fail7;
	}

	err = saa716x_ff_osd_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x FF OSD initialization failed");
		goto fail8;
	}

	return 0;

fail8:
	saa716x_ff_osd_exit(saa716x);
fail7:
	saa716x_ff_video_exit(saa716x);
fail6:
	saa716x_dvb_exit(saa716x);
fail5:
	SAA716x_EPWR(MSI, MSI_INT_ENA_CLR_H, MSI_INT_EXTINT_0);

	/* disable board power */
	saa716x_gpio_write(saa716x, TT_PREMIUM_GPIO_POWER_ENABLE, 0);

	vfree(sti7109->iobuf);
fail4:
	kfree(sti7109);
fail3:
	saa716x_i2c_exit(saa716x);
fail2:
	saa716x_pci_exit(saa716x);
fail1:
	kfree(saa716x);
fail0:
	return err;
}

static void __devexit saa716x_ff_pci_remove(struct pci_dev *pdev)
{
	struct saa716x_dev *saa716x = pci_get_drvdata(pdev);
	struct sti7109_dev *sti7109 = saa716x->priv;

	saa716x_ff_osd_exit(saa716x);

	saa716x_ff_video_exit(saa716x);

	saa716x_dvb_exit(saa716x);

	SAA716x_EPWR(MSI, MSI_INT_ENA_CLR_H, MSI_INT_EXTINT_0);

	/* disable board power */
	saa716x_gpio_write(saa716x, TT_PREMIUM_GPIO_POWER_ENABLE, 0);

	kfree(sti7109);

	saa716x_i2c_exit(saa716x);
	saa716x_pci_exit(saa716x);
	kfree(saa716x);
}

static irqreturn_t saa716x_ff_pci_irq(int irq, void *dev_id)
{
	struct saa716x_dev *saa716x	= (struct saa716x_dev *) dev_id;
	u32 msiStatusL;
	u32 msiStatusH;
	u32 phiISR;

	if (unlikely(saa716x == NULL)) {
		printk("%s: saa716x=NULL", __func__);
		return IRQ_NONE;
	}
#if 0
	dprintk(SAA716x_DEBUG, 1, "VI STAT 0=<%02x> 1=<%02x>, CTL 1=<%02x> 2=<%02x>",
		SAA716x_EPRD(VI0, INT_STATUS),
		SAA716x_EPRD(VI1, INT_STATUS),
		SAA716x_EPRD(VI0, INT_ENABLE),
		SAA716x_EPRD(VI1, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "FGPI STAT 0=<%02x> 1=<%02x>, CTL 1=<%02x> 2=<%02x>",
		SAA716x_EPRD(FGPI0, INT_STATUS),
		SAA716x_EPRD(FGPI1, INT_STATUS),
		SAA716x_EPRD(FGPI0, INT_ENABLE),
		SAA716x_EPRD(FGPI0, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "FGPI STAT 2=<%02x> 3=<%02x>, CTL 2=<%02x> 3=<%02x>",
		SAA716x_EPRD(FGPI2, INT_STATUS),
		SAA716x_EPRD(FGPI3, INT_STATUS),
		SAA716x_EPRD(FGPI2, INT_ENABLE),
		SAA716x_EPRD(FGPI3, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "AI STAT 0=<%02x> 1=<%02x>, CTL 0=<%02x> 1=<%02x>",
		SAA716x_EPRD(AI0, AI_STATUS),
		SAA716x_EPRD(AI1, AI_STATUS),
		SAA716x_EPRD(AI0, AI_CTL),
		SAA716x_EPRD(AI1, AI_CTL));

	dprintk(SAA716x_DEBUG, 1, "MSI STAT L=<%02x> H=<%02x>, CTL L=<%02x> H=<%02x>",
		SAA716x_EPRD(MSI, MSI_INT_STATUS_L),
		SAA716x_EPRD(MSI, MSI_INT_STATUS_H),
		SAA716x_EPRD(MSI, MSI_INT_ENA_L),
		SAA716x_EPRD(MSI, MSI_INT_ENA_H));

	dprintk(SAA716x_DEBUG, 1, "I2C STAT 0=<%02x> 1=<%02x>, CTL 0=<%02x> 1=<%02x>",
		SAA716x_EPRD(I2C_A, INT_STATUS),
		SAA716x_EPRD(I2C_B, INT_STATUS),
		SAA716x_EPRD(I2C_A, INT_ENABLE),
		SAA716x_EPRD(I2C_B, INT_ENABLE));

	dprintk(SAA716x_DEBUG, 1, "DCS STAT=<%02x>, CTL=<%02x>",
		SAA716x_EPRD(DCS, DCSC_INT_STATUS),
		SAA716x_EPRD(DCS, DCSC_INT_ENABLE));
#endif
	msiStatusL = SAA716x_EPRD(MSI, MSI_INT_STATUS_L);
	SAA716x_EPWR(MSI, MSI_INT_STATUS_CLR_L, msiStatusL);
	msiStatusH = SAA716x_EPRD(MSI, MSI_INT_STATUS_H);
	SAA716x_EPWR(MSI, MSI_INT_STATUS_CLR_H, msiStatusH);

	if (msiStatusL) {
		if (msiStatusL & MSI_INT_TAGACK_FGPI_2) {
			u32 fgpiStatus;
			u32 activeBuffer;

			fgpiStatus = SAA716x_EPRD(FGPI2, INT_STATUS);
			activeBuffer = (SAA716x_EPRD(BAM, BAM_FGPI2_DMA_BUF_MODE) >> 3) & 0x7;
			dprintk(SAA716x_DEBUG, 1, "fgpiStatus = %04X, buffer = %d",
				fgpiStatus, activeBuffer);
			if (activeBuffer > 0)
				activeBuffer -= 1;
			else
				activeBuffer = 7;
			if (saa716x->fgpi[2].dma_buf[activeBuffer].mem_virt) {
				u8 * data = (u8 *)saa716x->fgpi[2].dma_buf[activeBuffer].mem_virt;
				dprintk(SAA716x_DEBUG, 1, "%02X%02X%02X%02X",
					data[0], data[1], data[2], data[3]);
				dvb_dmx_swfilter_packets(&saa716x->saa716x_adap[0].demux, data, 348);
			}
			if (fgpiStatus) {
				SAA716x_EPWR(FGPI2, INT_CLR_STATUS, fgpiStatus);
			}
		}
		if (msiStatusL & MSI_INT_TAGACK_FGPI_3) {
			u32 fgpiStatus;
			u32 activeBuffer;

			fgpiStatus = SAA716x_EPRD(FGPI3, INT_STATUS);
			activeBuffer = (SAA716x_EPRD(BAM, BAM_FGPI3_DMA_BUF_MODE) >> 3) & 0x7;
			dprintk(SAA716x_DEBUG, 1, "fgpiStatus = %04X, buffer = %d",
				fgpiStatus, activeBuffer);
			if (activeBuffer > 0)
				activeBuffer -= 1;
			else
				activeBuffer = 7;
			if (saa716x->fgpi[3].dma_buf[activeBuffer].mem_virt) {
				u8 * data = (u8 *)saa716x->fgpi[3].dma_buf[activeBuffer].mem_virt;
				dprintk(SAA716x_DEBUG, 1, "%02X%02X%02X%02X",
					data[0], data[1], data[2], data[3]);
				dvb_dmx_swfilter_packets(&saa716x->saa716x_adap[1].demux, data, 348);
			}
			if (fgpiStatus) {
				SAA716x_EPWR(FGPI3, INT_CLR_STATUS, fgpiStatus);
			}
		}
	}
	if (msiStatusH) {
		//dprintk(SAA716x_INFO, 1, "msiStatusH: %08X", msiStatusH);
	}

	if (msiStatusH & MSI_INT_EXTINT_0) {

		struct sti7109_dev *sti7109 = saa716x->priv;

		phiISR = SAA716x_EPRD(PHI_1, FPGA_ADDR_EMI_ISR);
		//dprintk(SAA716x_INFO, 1, "interrupt status register: %08X", phiISR);
		if (phiISR & ISR_CMD_MASK) {

			u32 value;
			u32 length;
			/*dprintk(SAA716x_INFO, 1, "CMD interrupt source");*/

			value = SAA716x_EPRD(PHI_1, ADDR_CMD_DATA);
			value = __cpu_to_be32(value);
			length = (value >> 16) + 2;

			/*dprintk(SAA716x_INFO, 1, "CMD length: %d", length);*/

			if (length > MAX_RESULT_LEN) {
				dprintk(SAA716x_ERROR, 1, "CMD length %d > %d", length, MAX_RESULT_LEN);
				length = MAX_RESULT_LEN;
			}

			saa716x_phi_read(saa716x, ADDR_CMD_DATA, sti7109->result_data, length);
			sti7109->result_len = length;
			sti7109->result_avail = 1;
			wake_up(&sti7109->result_avail_wq);

			phiISR &= ~ISR_CMD_MASK;
			SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICLR, ISR_CMD_MASK);
		}

		if (phiISR & ISR_READY_MASK) {
			/*dprintk(SAA716x_INFO, 1, "READY interrupt source");*/
			sti7109->cmd_ready = 1;
			wake_up(&sti7109->cmd_ready_wq);
			phiISR &= ~ISR_READY_MASK;
			SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICLR, ISR_READY_MASK);
		}

		if (phiISR & ISR_BLOCK_MASK) {
			/*dprintk(SAA716x_INFO, 1, "BLOCK interrupt source");*/
			sti7109->block_done = 1;
			wake_up(&sti7109->block_done_wq);
			phiISR &= ~ISR_BLOCK_MASK;
			SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICLR, ISR_BLOCK_MASK);
		}

		if (phiISR & ISR_DATA_MASK) {
			/*dprintk(SAA716x_INFO, 1, "DATA interrupt source");*/
			sti7109->data_ready = 1;
			wake_up(&sti7109->data_ready_wq);
			phiISR &= ~ISR_DATA_MASK;
			SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICLR, ISR_DATA_MASK);
		}

		if (phiISR & ISR_BOOT_FINISH_MASK) {
			/*dprintk(SAA716x_INFO, 1, "BOOT FINISH interrupt source");*/
			sti7109->boot_finished = 1;
			wake_up(&sti7109->boot_finish_wq);
			phiISR &= ~ISR_BOOT_FINISH_MASK;
			SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICLR, ISR_BOOT_FINISH_MASK);
		}

		if (phiISR & ISR_AUDIO_PTS_MASK) {
			u8 data[8];

			saa716x_phi_read(saa716x, ADDR_AUDIO_PTS, data, 8);
			sti7109->audio_pts = ((u64) (data[3] & 0x01) << 32)
					    | (u64) (data[4] << 24)
					    | (u64) (data[5] << 16)
					    | (u64) (data[6] << 8)
					    | (u64) (data[7]);

			phiISR &= ~ISR_AUDIO_PTS_MASK;
			SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICLR, ISR_AUDIO_PTS_MASK);

			/*dprintk(SAA716x_INFO, 1, "AUDIO PTS: %llu", sti7109->audio_pts);*/
		}

		if (phiISR & ISR_VIDEO_PTS_MASK) {
			u8 data[8];

			saa716x_phi_read(saa716x, ADDR_VIDEO_PTS, data, 8);
			sti7109->video_pts = ((u64) (data[3] & 0x01) << 32)
					    | (u64) (data[4] << 24)
					    | (u64) (data[5] << 16)
					    | (u64) (data[6] << 8)
					    | (u64) (data[7]);

			phiISR &= ~ISR_VIDEO_PTS_MASK;
			SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICLR, ISR_VIDEO_PTS_MASK);

			/*dprintk(SAA716x_INFO, 1, "VIDEO PTS: %llu", sti7109->video_pts);*/
		}

		if (phiISR & ISR_CURRENT_STC_MASK) {
			u8 data[8];

			saa716x_phi_read(saa716x, ADDR_CURRENT_STC, data, 8);
			sti7109->current_stc = ((u64) (data[3] & 0x01) << 32)
					      | (u64) (data[4] << 24)
					      | (u64) (data[5] << 16)
					      | (u64) (data[6] << 8)
					      | (u64) (data[7]);

			phiISR &= ~ISR_CURRENT_STC_MASK;
			SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICLR, ISR_CURRENT_STC_MASK);

			/*dprintk(SAA716x_INFO, 1, "CURRENT STC: %llu", sti7109->current_stc);*/
		}

		if (phiISR & ISR_REMOTE_EVENT_MASK) {
			u8 data[4];

			saa716x_phi_read(saa716x, ADDR_REMOTE_EVENT, data, 4);
			sti7109->remote_event = (data[0] << 24)
					      | (data[1] << 16)
					      | (data[2] << 8)
					      | (data[3]);

			phiISR &= ~ISR_REMOTE_EVENT_MASK;
			SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICLR, ISR_REMOTE_EVENT_MASK);

			dprintk(SAA716x_INFO, 1, "REMOTE EVENT: %u", sti7109->remote_event);
		}

		if (phiISR & ISR_FIFO_EMPTY_MASK) {
			u32 fifoCtrl;
			u32 fifoStat;
			u16 fifoSize;
			u16 fifoUsage;
			u16 fifoFree;
			int len;

			/*dprintk(SAA716x_INFO, 1, "FIFO EMPTY interrupt source");*/
			fifoCtrl = SAA716x_EPRD(PHI_1, FPGA_ADDR_FIFO_CTRL);
			fifoCtrl &= ~0x4;
			SAA716x_EPWR(PHI_1, FPGA_ADDR_FIFO_CTRL, fifoCtrl);
			fifoStat = SAA716x_EPRD(PHI_1, FPGA_ADDR_FIFO_STAT);
			fifoSize = (u16) (fifoStat >> 16);
			fifoUsage = (u16) fifoStat;
			fifoFree = fifoSize - fifoUsage;
			spin_lock(&sti7109->tsout.lock);
			len = dvb_ringbuffer_avail(&sti7109->tsout);
			if (len > fifoFree)
				len = fifoFree;
			if (len >= TS_SIZE)
			{
				while (len >= TS_SIZE)
				{
					dvb_ringbuffer_read(&sti7109->tsout, sti7109->tsbuf, (size_t) TS_SIZE);
					saa716x_phi_write_fifo(saa716x, sti7109->tsbuf, TS_SIZE);
					len -= TS_SIZE;
				}
				wake_up(&sti7109->tsout.queue);
				fifoCtrl |= 0x4;
				SAA716x_EPWR(PHI_1, FPGA_ADDR_FIFO_CTRL, fifoCtrl);
			}
			spin_unlock(&sti7109->tsout.lock);
			phiISR &= ~ISR_FIFO_EMPTY_MASK;
		}

		if (phiISR) {
			dprintk(SAA716x_INFO, 1, "unknown interrupt source");
			SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICLR, phiISR);
		}
	}

	return IRQ_HANDLED;
}

static int load_config_s26400(struct saa716x_dev *saa716x)
{
	int ret = 0;

	return ret;
}

#define SAA716x_MODEL_S2_6400_DUAL	"Technotrend S2 6400 Dual S2 Premium"
#define SAA716x_DEV_S2_6400_DUAL	"2x DVB-S/S2 + Hardware decode"

static struct stv090x_config tt6400_stv090x_config = {
	.device			= STV0900,
	.demod_mode		= STV090x_DUAL,
	.clk_mode		= STV090x_CLK_EXT,

	.xtal			= 13500000,
	.address		= 0x68,

	.ts1_mode		= STV090x_TSMODE_SERIAL_CONTINUOUS,
	.ts2_mode		= STV090x_TSMODE_SERIAL_CONTINUOUS,
	.ts1_clk		= 90000000,
	.ts2_clk		= 90000000,

	.repeater_level		= STV090x_RPTLEVEL_16,

	.tuner_init		= NULL,
	.tuner_set_mode		= NULL,
	.tuner_set_frequency	= NULL,
	.tuner_get_frequency	= NULL,
	.tuner_set_bandwidth	= NULL,
	.tuner_get_bandwidth	= NULL,
	.tuner_set_bbgain	= NULL,
	.tuner_get_bbgain	= NULL,
	.tuner_set_refclk	= NULL,
	.tuner_get_status	= NULL,
};

static struct stv6110x_config tt6400_stv6110x_config = {
	.addr			= 0x60,
	.refclk			= 27000000,
	.clk_div		= 2,
};

static struct isl6423_config tt6400_isl6423_config[2] = {
	{
		.current_max		= SEC_CURRENT_515m,
		.curlim			= SEC_CURRENT_LIM_ON,
		.mod_extern		= 1,
		.addr			= 0x09,
	},
	{
		.current_max		= SEC_CURRENT_515m,
		.curlim			= SEC_CURRENT_LIM_ON,
		.mod_extern		= 1,
		.addr			= 0x08,
	}
};


static int saa716x_s26400_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x	= adapter->saa716x;
	struct saa716x_i2c *i2c		= saa716x->i2c;
	struct i2c_adapter *i2c_adapter	= &i2c[SAA716x_I2C_BUS_A].i2c_adapter;

	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) SAA716x frontend Init", count);
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);

	if (count == 0 || count == 1) {
		adapter->fe = dvb_attach(stv090x_attach,
					 &tt6400_stv090x_config,
					 i2c_adapter,
					 STV090x_DEMODULATOR_0 + count);

		if (adapter->fe) {
			struct stv6110x_devctl *ctl;
			ctl = dvb_attach(stv6110x_attach,
					 adapter->fe,
					 &tt6400_stv6110x_config,
					 i2c_adapter);

			tt6400_stv090x_config.tuner_init	  = ctl->tuner_init;
			tt6400_stv090x_config.tuner_set_mode	  = ctl->tuner_set_mode;
			tt6400_stv090x_config.tuner_set_frequency = ctl->tuner_set_frequency;
			tt6400_stv090x_config.tuner_get_frequency = ctl->tuner_get_frequency;
			tt6400_stv090x_config.tuner_set_bandwidth = ctl->tuner_set_bandwidth;
			tt6400_stv090x_config.tuner_get_bandwidth = ctl->tuner_get_bandwidth;
			tt6400_stv090x_config.tuner_set_bbgain	  = ctl->tuner_set_bbgain;
			tt6400_stv090x_config.tuner_get_bbgain	  = ctl->tuner_get_bbgain;
			tt6400_stv090x_config.tuner_set_refclk	  = ctl->tuner_set_refclk;
			tt6400_stv090x_config.tuner_get_status	  = ctl->tuner_get_status;

			dvb_attach(isl6423_attach,
				   adapter->fe,
				   i2c_adapter,
				   &tt6400_isl6423_config[count]);

		} else {
			dvb_frontend_detach(adapter->fe);
			adapter->fe = NULL;
		}
		if (adapter->fe == NULL) {
			dprintk(SAA716x_ERROR, 1, "A frontend driver was not found for [%04x:%04x subsystem [%04x:%04x]\n",
				saa716x->pdev->vendor,
				saa716x->pdev->device,
				saa716x->pdev->subsystem_vendor,
				saa716x->pdev->subsystem_device);

		} else {
			if (dvb_register_frontend(&adapter->dvb_adapter, adapter->fe)) {
				dprintk(SAA716x_ERROR, 1, "Frontend registration failed!\n");
				dvb_frontend_detach(adapter->fe);
				adapter->fe = NULL;
			}
		}
	}
	return 0;
}

static struct saa716x_config saa716x_s26400_config = {
	.model_name		= SAA716x_MODEL_S2_6400_DUAL,
	.dev_type		= SAA716x_DEV_S2_6400_DUAL,
	.boot_mode		= SAA716x_EXT_BOOT,
	.load_config		= &load_config_s26400,
	.adapters		= 2,
	.frontend_attach	= saa716x_s26400_frontend_attach,
	.irq_handler		= saa716x_ff_pci_irq,
	.i2c_rate		= SAA716x_I2C_RATE_100,

	.adap_config		= {
		{
			/* Adapter 0 */
			.ts_port = 2
		},{
			/* Adapter 1 */
			.ts_port = 3
		}
	}
};


static struct pci_device_id saa716x_ff_pci_table[] = {

	MAKE_ENTRY(TECHNOTREND, S2_6400_DUAL_S2_PREMIUM, SAA7160, &saa716x_s26400_config), /* S2 6400 Dual */
	{ }
};
MODULE_DEVICE_TABLE(pci, saa716x_ff_pci_table);

static struct pci_driver saa716x_ff_pci_driver = {
	.name			= DRIVER_NAME,
	.id_table		= saa716x_ff_pci_table,
	.probe			= saa716x_ff_pci_probe,
	.remove			= saa716x_ff_pci_remove,
};

static int __devinit saa716x_ff_init(void)
{
	return pci_register_driver(&saa716x_ff_pci_driver);
}

static void __devexit saa716x_ff_exit(void)
{
	return pci_unregister_driver(&saa716x_ff_pci_driver);
}

module_init(saa716x_ff_init);
module_exit(saa716x_ff_exit);

MODULE_DESCRIPTION("SAA716x FF driver");
MODULE_AUTHOR("Manu Abraham");
MODULE_LICENSE("GPL");
