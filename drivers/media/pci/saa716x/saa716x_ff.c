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

#include <linux/i2c.h>
#include "saa716x_priv.h"
#include "saa716x_vip.h"
#include "saa716x_aip.h"
#include "saa716x_msi.h"
#include "saa716x_reg.h"
#include "saa716x_ff.h"
#include "saa716x_adap.h"
#include "saa716x_gpio.h"
#include "saa716x_phi.h"

#include <linux/dvb/osd.h>

unsigned int verbose;
module_param(verbose, int, 0644);
MODULE_PARM_DESC(verbose, "verbose startup messages, default is 1 (yes)");

unsigned int int_type;
module_param(int_type, int, 0644);
MODULE_PARM_DESC(int_type, "force Interrupt Handler type: 0=INT-A, 1=MSI, 2=MSI-X. default INT-A mode");

#define DRIVER_NAME	"SAA716x FF"

static int saa716x_ff_fpga_init(struct saa716x_dev *saa716x);
static int saa716x_ff_st7109_init(struct saa716x_dev *saa716x);
static int saa716x_ff_osd_init(struct saa716x_dev *saa716x);
static int saa716x_ff_osd_exit(struct saa716x_dev *saa716x);
static int sti7109_send_cmd(struct sti7109_dev *sti7109, const u8 *data, int length);

static int __devinit saa716x_ff_pci_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
	struct saa716x_dev *saa716x;
	struct sti7109_dev *sti7109;
	int err = 0;
	u32 value;

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

	/* prepare the sti7109 device struct */
	sti7109 = kzalloc(sizeof(struct sti7109_dev), GFP_KERNEL);
	if (!sti7109) {
		dprintk(SAA716x_ERROR, 1, "SAA716x: out of memory");
		goto fail3;
	}

	sti7109->dev	= saa716x;
	saa716x->priv	= sti7109;

	saa716x_gpio_set_output(saa716x, TT_PREMIUM_GPIO_POWER_ENABLE);
	saa716x_gpio_set_output(saa716x, TT_PREMIUM_GPIO_RESET_BACKEND);
	saa716x_gpio_set_output(saa716x, TT_PREMIUM_GPIO_FPGA_CS0);
	saa716x_gpio_set_output(saa716x, TT_PREMIUM_GPIO_FPGA_CS1);
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
		goto fail4;
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
		goto fail4;
	}

	err = saa716x_dvb_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x DVB initialization failed");
		goto fail5;
	}

	err = saa716x_ff_osd_init(saa716x);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "SAA716x FF OSD initialization failed");
		goto fail6;
	}

	if (0) {
		u8 data[32];

		memset(data, 0, sizeof(data));
		data[0] = 0;
		data[1] = 13;
		data[2] = 0;
		// command group
		data[3] = 4;
		// command id
		data[4] = 0;
		data[5] = 10; // create display
		// width
		data[6] = 0;
		data[7] = 0;
		data[8] = 0x07;
		data[9] = 0x80;
		// height
		data[10] = 0;
		data[11] = 0;
		data[12] = 0x04;
		data[13] = 0x38;
		// color type
		data[14] = 4;
		sti7109_send_cmd(sti7109, data, 15);

		msleep(100);

		memset(data, 0, sizeof(data));
		data[0] = 0;
		data[1] = 28;
		data[2] = 0;
		// command group
		data[3] = 4;
		// command id
		data[4] = 0;
		data[5] = 71; // draw rectangle
		// display handle
		data[6] = 0;
		data[7] = 0;
		data[8] = 0;
		data[9] = 0;
		// x
		data[10] = 0;
		data[11] = 0;
		data[12] = 0;
		data[13] = 100;
		// y
		data[14] = 0;
		data[15] = 0;
		data[16] = 0;
		data[17] = 200;
		// w
		data[18] = 0;
		data[19] = 0;
		data[20] = 0x02;
		data[21] = 0x00;
		// h
		data[22] = 0;
		data[23] = 0;
		data[24] = 0x01;
		data[25] = 0x00;
		// color
		data[26] = 0xFF;
		data[27] = 0xFF;
		data[28] = 0x00;
		data[29] = 0x00;
		sti7109_send_cmd(sti7109, data, 30);

		msleep(100);

		memset(data, 0, sizeof(data));
		data[0] = 0;
		data[1] = 8;
		data[2] = 0;
		// command group
		data[3] = 4;
		// command id
		data[4] = 0;
		data[5] = 15; // render display
		// handle
		data[6] = 0;
		data[7] = 0;
		data[8] = 0;
		data[9] = 0;
		sti7109_send_cmd(sti7109, data, 10);
	}

	return 0;

fail6:
	saa716x_ff_osd_exit(saa716x);
fail5:
	saa716x_dvb_exit(saa716x);
fail4:
	SAA716x_EPWR(MSI, MSI_INT_ENA_CLR_H, MSI_INT_EXTINT_0);

	/* disable board power */
	saa716x_gpio_write(saa716x, TT_PREMIUM_GPIO_POWER_ENABLE, 0);

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
	uint32_t msiStatusH;
	uint32_t phiISR;

	if (unlikely(saa716x == NULL)) {
		printk("%s: saa716x=NULL", __func__);
		return IRQ_NONE;
	}

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

	SAA716x_EPWR(MSI, MSI_INT_STATUS_CLR_L, SAA716x_EPRD(MSI, MSI_INT_STATUS_L));

	msiStatusH = SAA716x_EPRD(MSI, MSI_INT_STATUS_H);
	SAA716x_EPWR(MSI, MSI_INT_STATUS_CLR_H, msiStatusH);

	if (msiStatusH & MSI_INT_EXTINT_0) {

		struct sti7109_dev *sti7109 = saa716x->priv;

		phiISR = SAA716x_EPRD(PHI_1, FPGA_ADDR_EMI_ISR);
		dprintk(SAA716x_INFO, 1, "interrupt status register: %08X", phiISR);
		if (phiISR & ISR_CMD_MASK) {

			u32 value;
			u32 length;
			dprintk(SAA716x_INFO, 1, "CMD interrupt source");

			value = SAA716x_EPRD(PHI_1, 0);
			value = __cpu_to_be32(value);
			length = (value >> 16) + 2;

			dprintk(SAA716x_INFO, 1, "CMD length: %d", length);

			if (length > MAX_RESULT_LEN) {
				dprintk(SAA716x_ERROR, 1, "CMD length %d > %d", length, MAX_RESULT_LEN);
				length = MAX_RESULT_LEN;
			}

			saa716x_phi_read(saa716x, 0, sti7109->result_data, length);
			sti7109->result_len = length;
			sti7109->result_avail = 1;
			wake_up(&sti7109->result_avail_wq);

			SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICLR, ISR_CMD_MASK);
		}

		if (phiISR & ISR_READY_MASK) {
			dprintk(SAA716x_INFO, 1, "READY interrupt source");
			sti7109->cmd_ready = 1;
			wake_up(&sti7109->cmd_ready_wq);
			SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICLR, ISR_READY_MASK);
		}

		if (phiISR & ISR_BLOCK_MASK) {
			dprintk(SAA716x_INFO, 1, "BLOCK interrupt source");
			sti7109->block_done = 1;
			wake_up(&sti7109->block_done_wq);
			SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICLR, ISR_BLOCK_MASK);
		}

		if (phiISR & ISR_DATA_MASK) {
			dprintk(SAA716x_INFO, 1, "DATA interrupt source");
			sti7109->data_ready = 1;
			wake_up(&sti7109->data_ready_wq);
			SAA716x_EPWR(PHI_1, FPGA_ADDR_EMI_ICLR, ISR_DATA_MASK);
		}
	}

	return IRQ_HANDLED;
}

static void saa716x_spi_write(struct saa716x_dev *saa716x, const uint8_t * data, int length)
{
	int i;
	u32 value;
	int rounds;

	for (i = 0; i < length; i++) {
		SAA716x_EPWR(SPI, SPI_DATA, data[i]);
		rounds = 0;
		value = SAA716x_EPRD(SPI, SPI_STATUS);

		while ((value & SPI_TRANSFER_FLAG) == 0 && rounds < 5000) {
			value = SAA716x_EPRD(SPI, SPI_STATUS);
			rounds++;
		}
	}
}

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

	writtenBlock++;
	writtenBlock |= 0x80000000;
	SAA716x_EPWR(PHI_1, 0x3ff8, writtenBlock);

	dprintk(SAA716x_INFO, 1, "SAA716x download ST7109 firmware done");

	release_firmware(fw);

	return 0;
}

static int sti7109_send_cmd(struct sti7109_dev * sti7109, const uint8_t * data, int length)
{
	struct saa716x_dev * saa716x = sti7109->dev;
	unsigned long timeout;

	timeout = 10 * HZ;
	timeout = wait_event_interruptible_timeout(sti7109->cmd_ready_wq,
						   sti7109->cmd_ready == 1,
						   timeout);

	if (timeout == -ERESTARTSYS || sti7109->cmd_ready == 0) {
		if (timeout == -ERESTARTSYS) {
			/* a signal arrived */
			return -ERESTARTSYS;
		}
		dprintk(SAA716x_ERROR, 1, "timed out waiting for command ready");
		return -EIO;
	}

	sti7109->cmd_ready = 0;
	sti7109->result_avail = 0;
	saa716x_phi_write(saa716x, 0x0000, data, length);
	SAA716x_EPWR(PHI_1, FPGA_ADDR_PHI_ISET, ISR_CMD_MASK);
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
		timeout = wait_event_interruptible_timeout(sti7109->block_done_wq,
							   sti7109->block_done == 1,
							   timeout);

		if (timeout == -ERESTARTSYS || sti7109->block_done == 0) {
			if (timeout == -ERESTARTSYS) {
				/* a signal arrived */
				return -ERESTARTSYS;
			}
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

	if (cmd == OSD_RAW_CMD)
		return sti7109_raw_cmd(sti7109, (osd_raw_cmd_t *) parg);
	else if (cmd == OSD_RAW_DATA)
		return sti7109_raw_data(sti7109, (osd_raw_data_t *) parg);

	return -EINVAL;
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

static int saa716x_ff_osd_init(struct saa716x_dev *saa716x)
{
	struct saa716x_adapter *saa716x_adap	= saa716x->saa716x_adap;
	struct sti7109_dev *sti7109		= saa716x->priv;

	init_waitqueue_head(&sti7109->cmd_ready_wq);
	sti7109->cmd_ready = 0;

	init_waitqueue_head(&sti7109->result_avail_wq);
	sti7109->result_avail = 0;

	sti7109->data_handle = 0;
	init_waitqueue_head(&sti7109->data_ready_wq);
	sti7109->data_ready = 0;
	init_waitqueue_head(&sti7109->block_done_wq);
	sti7109->block_done = 0;

	dvb_register_device(&saa716x_adap->dvb_adapter,
			    &sti7109->osd_dev,
			    &dvbdev_osd,
			    sti7109,
			    DVB_DEVICE_OSD);
	return 0;
}

static int saa716x_ff_osd_exit(struct saa716x_dev *saa716x)
{
	struct sti7109_dev *sti7109 = saa716x->priv;

	dvb_unregister_device(sti7109->osd_dev);
	return 0;
}

static int load_config_s26400(struct saa716x_dev *saa716x)
{
	int ret = 0;

	return ret;
}

#define SAA716x_MODEL_S2_6400_DUAL	"Technotrend S2 6400 Dual S2 Premium"
#define SAA716x_DEV_S2_6400_DUAL	"2x DVB-S/S2 + Hardware decode"

static int saa716x_s26400_frontend_attach(struct saa716x_adapter *adapter, int count)
{
	struct saa716x_dev *saa716x = adapter->saa716x;
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) SAA716x frontend Init", count);
	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Device ID=%02x", count, saa716x->pdev->subsystem_device);

	return -ENODEV;
}

static struct saa716x_config saa716x_s26400_config = {
	.model_name		= SAA716x_MODEL_S2_6400_DUAL,
	.dev_type		= SAA716x_DEV_S2_6400_DUAL,
	.boot_mode		= SAA716x_EXT_BOOT,
	.load_config		= &load_config_s26400,
	.adapters		= 2,
	.frontend_attach	= saa716x_s26400_frontend_attach,
	.irq_handler		= saa716x_ff_pci_irq,

	.adap_config		= {
		{
			/* Adapter 0 */
		},{
			/* Adapter 1 */
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
