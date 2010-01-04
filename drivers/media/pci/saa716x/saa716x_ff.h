#ifndef __SAA716x_FF_H
#define __SAA716x_FF_H

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

#define ISR_CMD_MASK		0x0001 /* interrupt source for normal cmds (osd, fre, av, ...) */
#define ISR_READY_MASK		0x0002 /* interrupt source for command acknowledge */
#define ISR_BLOCK_MASK		0x0004 /* interrupt source for single block transfers and acknowledge */
#define ISR_DATA_MASK		0x0008 /* interrupt source for data transfer acknowledge */

#define ADDR_CMD_DATA		0x0000 /* address for cmd data in fpga dpram */
#define ADDR_BLOCK_DATA		0x0600 /* address for block data in fpga dpram */

#define SIZE_CMD_DATA		0x0600 /* maximum size for command data (1,5 kB) */
#define SIZE_BLOCK_DATA		0x3800 /* maximum size for block data (14 kB) */

#define SIZE_BLOCK_HEADER	8      /* block header size */

#define MAX_RESULT_LEN		64

/* place to store all the necessary device information */
struct sti7109_dev {
	struct saa716x_dev	*dev;
	struct dvb_device	*osd_dev;

	wait_queue_head_t	cmd_ready_wq;
	int			cmd_ready;

	wait_queue_head_t	result_avail_wq;
	int			result_avail;
	uint8_t			result_data[MAX_RESULT_LEN];
	uint32_t		result_len;

	uint16_t		data_handle;
	wait_queue_head_t	data_ready_wq;
	int			data_ready;
	wait_queue_head_t	block_done_wq;
	int			block_done;
};

#endif /* __SAA716x_FF_H */
