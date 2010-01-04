#ifndef __SAA716x_FF_H
#define __SAA716x_FF_H

#include "dvb_filter.h"
#include "dvb_ringbuffer.h"

#define TECHNOTREND			0x13c2
#define S2_6400_DUAL_S2_PREMIUM		0x3009

#define TT_PREMIUM_GPIO_POWER_ENABLE	27
#define TT_PREMIUM_GPIO_RESET_BACKEND	26
#define TT_PREMIUM_GPIO_FPGA_CS1	17
#define TT_PREMIUM_GPIO_FPGA_CS0	16
#define TT_PREMIUM_GPIO_FPGA_PROGRAMN	15
#define TT_PREMIUM_GPIO_FPGA_DONE	14
#define TT_PREMIUM_GPIO_FPGA_INITN	13

/* fpga interrupt register addresses */
#define FPGA_ADDR_PHI_ICTRL	0x8000 /* PHI General control of the PC => STB interrupt controller */
#define FPGA_ADDR_PHI_ISR	0x8010 /* PHI Interrupt Status Register */
#define FPGA_ADDR_PHI_ISET	0x8020 /* PHI Interrupt Set Register */
#define FPGA_ADDR_PHI_ICLR	0x8030 /* PHI Interrupt Clear Register */
#define FPGA_ADDR_EMI_ICTRL	0x8100 /* EMI General control of the STB => PC interrupt controller */
#define FPGA_ADDR_EMI_ISR	0x8110 /* EMI Interrupt Status Register */
#define FPGA_ADDR_EMI_ISET	0x8120 /* EMI Interrupt Set Register */
#define FPGA_ADDR_EMI_ICLR	0x8130 /* EMI Interrupt Clear Register */

#define FPGA_ADDR_TSR_CTRL	0x8200 /* TS router control register */
#define FPGA_ADDR_TSR_MUX1	0x8210 /* TS multiplexer 1 selection register */
#define FPGA_ADDR_TSR_MUX2	0x8220 /* TS multiplexer 2 selection register */
#define FPGA_ADDR_TSR_MUX3	0x8230 /* TS multiplexer 3 selection register */
#define FPGA_ADDR_TSR_MUXCI1	0x8240 /* TS multiplexer CI 1 selection register */
#define FPGA_ADDR_TSR_MUXCI2	0x8250 /* TS multiplexer CI 2 selection register */

#define FPGA_ADDR_FIFO_CTRL	0x8300 /* FIFO control register */
#define FPGA_ADDR_FIFO_STAT	0x8310 /* FIFO status register */

#define ISR_CMD_MASK		0x0001 /* interrupt source for normal cmds (osd, fre, av, ...) */
#define ISR_READY_MASK		0x0002 /* interrupt source for command acknowledge */
#define ISR_BLOCK_MASK		0x0004 /* interrupt source for single block transfers and acknowledge */
#define ISR_DATA_MASK		0x0008 /* interrupt source for data transfer acknowledge */
#define ISR_BOOT_FINISH_MASK	0x0010 /* interrupt source for boot finish indication */
#define ISR_AUDIO_PTS_MASK	0x0020 /* interrupt source for audio PTS */
#define ISR_VIDEO_PTS_MASK	0x0040 /* interrupt source for video PTS */
#define ISR_CURRENT_STC_MASK	0x0080 /* interrupt source for current system clock */
#define ISR_REMOTE_EVENT_MASK	0x0100 /* interrupt source for remote events */
#define ISR_FIFO_EMPTY_MASK	0x8000 /* interrupt source for FIFO empty */

#define ADDR_CMD_DATA		0x0000 /* address for cmd data in fpga dpram */
#define ADDR_BLOCK_DATA		0x0600 /* address for block data in fpga dpram */
#define ADDR_AUDIO_PTS		0x3E00 /* address for audio PTS (64 Bits) */
#define ADDR_VIDEO_PTS		0x3E08 /* address for video PTS (64 Bits) */
#define ADDR_CURRENT_STC	0x3E10 /* address for system clock (64 Bits) */
#define ADDR_REMOTE_EVENT	0x3F00 /* address for remote events (32 Bits) */

#define SIZE_CMD_DATA		0x0600 /* maximum size for command data (1,5 kB) */
#define SIZE_BLOCK_DATA		0x3800 /* maximum size for block data (14 kB) */

#define SIZE_BLOCK_HEADER	8      /* block header size */

#define MAX_RESULT_LEN		256

#define TSOUT_LEN		(1024 * TS_SIZE)
#define TSBUF_LEN		(8 * 1024)

/* place to store all the necessary device information */
struct sti7109_dev {
	struct saa716x_dev	*dev;
	struct dvb_device	*osd_dev;
	struct dvb_device	*video_dev;

	void			*iobuf;	 /* memory for all buffers */
	struct dvb_ringbuffer	tsout;   /* buffer for TS output */
	u8			*tsbuf;  /* temp ts buffer */

	wait_queue_head_t	boot_finish_wq;
	int			boot_finished;

	wait_queue_head_t	cmd_ready_wq;
	int			cmd_ready;

	wait_queue_head_t	result_avail_wq;
	int			result_avail;
	u8			result_data[MAX_RESULT_LEN];
	u32			result_len;

	u16			data_handle;
	wait_queue_head_t	data_ready_wq;
	int			data_ready;
	wait_queue_head_t	block_done_wq;
	int			block_done;

	struct mutex		cmd_lock;
	struct mutex		data_lock;

	u64			audio_pts;
	u64			video_pts;
	u64			current_stc;
	u32			remote_event;
};

#endif /* __SAA716x_FF_H */
