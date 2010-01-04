#ifndef __SAA716x_REG_H
#define __SAA716x_REG_H

/* BAR = 17 bits */
/*
	VI0	0x00000000
	VI1	0x00001000
	FGPI0	0x00002000
	FGPI1	0x00003000
	FGPI2	0x00004000
	FGPI3	0x00005000
	AI0	0x00006000
	AI1	0x00007000
	BAM	0x00008000
	MMU	0x00009000
	MSI	0x0000a000
	I2C_B	0x0000b000
	I2C_A	0x0000c000
	SPI	0x0000d000
	GPIO	0x0000e000
	PHI_0	0x0000f000
	CGU	0x00013000
	DCS	0x00014000
	GREG	0x00012000
*/

/* -------------- VIP Registers -------------- */

#define VI0				0x00000000
#define VI1				0x00001000

#define VI_MODE				0x000
#define VID_CFEN			(0x00000003 << 30)
#define VID_OSM				(0x00000001 << 29)
#define VID_FSEQ			(0x00000001 << 28)
#define AUX_CFEN			(0x00000003 << 26)
#define AUX_OSM				(0x00000001 << 25)
#define AUX_FSEQ			(0x00000001 << 24)
#define AUX_ANC_DATA			(0x00000003 << 22)
#define AUX_ANC_RAW			(0x00000001 << 21)
#define RST_ON_ERR			(0x00000001 << 17)
#define SOFT_RESET			(0x00000001 << 16)
#define IFF_CLAMP			(0x00000001 << 14)
#define IFF_MODE			(0x00000003 << 12)
#define DFF_CLAMP			(0x00000001 << 10)
#define DFF_MODE			(0x00000003 <<  8)
#define HSP_CLAMP			(0x00000001 <<  3)
#define HSP_RGB				(0x00000001 <<  2)
#define HSP_MODE			(0x00000003 <<  0)

#define RCRB_CTRL			0x004
#define RCRB_CFG_ADDR			0x008
#define RCRB_CFG_EXT_ADDR		0x00c
#define RCRB_IO_ADDR			0x010
#define RCRB_MEM_LADDR			0x014
#define RCRB_MEM_UADDR			0x018
#define RCRB_DATA			0x01c
#define RCRB_MASK			0x020
#define RCRB_MSG_HDR			0x040
#define RCRB_MSG_PL0			0x044
#define RCRB_MSG_PL1			0x048

#define ID_MASK0			0x020
#define VI_ID_MASK_0			(0x000000ff <<  8)
#define VI_DATA_ID_0			(0x000000ff <<  0)

#define ID_MASK1			0x024
#define VI_ID_MASK_1			(0x000000ff <<  8)
#define VI_DATA_ID_1			(0x000000ff <<  0)

#define VIP_LINE_THRESH			0x040
#define VI_LCTHR			(0x000007ff <<  0)

#define VIN_FORMAT			0x100
#define VI_VSRA				(0x00000003 << 30)
#define VI_SYNCHD			(0x00000001 << 25)
#define VI_DUAL_STREAM			(0x00000001 << 24)
#define VI_NHDAUX			(0x00000001 << 20)
#define VI_NPAR				(0x00000001 << 19)
#define VI_VSEL				(0x00000003 << 14)
#define VI_TWOS				(0x00000001 << 13)
#define VI_TPG				(0x00000001 << 12)
#define VI_FREF				(0x00000001 << 10)
#define VI_FTGL				(0x00000001 <<  9)
#define VI_SF				(0x00000001 <<  3)
#define VI_FZERO			(0x00000001 <<  2)
#define VI_REVS				(0x00000001 <<  1)
#define VI_REHS				(0x00000001 <<  0)

#define TC76543210			0x800
#define TCFEDCBA98			0x804
#define PHYCFG				0x900
#define CONFIG				0xfd4
#define INT_ENABLE_CLR			0xfd8
#define INT_ENABLE_SET			0xfdc


#define INT_STATUS			0xfe0
#define VI_STAT_FID_AUX			(0x00000001 << 31)
#define VI_STAT_FID_VID			(0x00000001 << 30)
#define VI_STAT_FID_VPI			(0x00000001 << 29)
#define VI_STAT_LINE_COUNT		(0x00000fff << 16)
#define VI_STAT_AUX_OVRFLW		(0x00000001 <<  9)
#define VI_STAT_VID_OVRFLW		(0x00000001 <<  8)
#define VI_STAT_WIN_SEQBRK		(0x00000001 <<  7)
#define VI_STAT_FID_SEQBRK		(0x00000001 <<  6)
#define VI_STAT_LINE_THRESH		(0x00000001 <<  5)
#define VI_STAT_AUX_WRAP		(0x00000001 <<  4)
#define VI_STAT_AUX_START_IN		(0x00000001 <<  3)
#define VI_STAT_AUX_END_OUT		(0x00000001 <<  2)
#define VI_STAT_VID_START_IN		(0x00000001 <<  1)
#define VI_STAT_VID_END_OUT		(0x00000001 <<  0)

#define INT_ENABLE			0xfe4
#define VI_ENABLE_AUX_OVRFLW		(0x00000001 <<  9)
#define VI_ENABLE_VID_OVRFLW		(0x00000001 <<  8)
#define VI_ENABLE_WIN_SEQBRK		(0x00000001 <<  7)
#define VI_ENABLE_FID_SEQBRK		(0x00000001 <<  6)
#define VI_ENABLE_LINE_THRESH		(0x00000001 <<  5)
#define VI_ENABLE_AUX_WRAP		(0x00000001 <<  4)
#define VI_ENABLE_AUX_START_IN		(0x00000001 <<  3)
#define VI_ENABLE_AUX_END_OUT		(0x00000001 <<  2)
#define VI_ENABLE_VID_START_IN		(0x00000001 <<  1)
#define VI_ENABLE_VID_END_OUT		(0x00000001 <<  0)

#define INT_CLR_STATUS			0xfe8
#define VI_CLR_STATUS_AUX_OVRFLW	(0x00000001 <<  9)
#define VI_CLR_STATUS_VID_OVRFLW	(0x00000001 <<  8)
#define VI_CLR_STATUS_WIN_SEQBRK	(0x00000001 <<  7)
#define VI_CLR_STATUS_FID_SEQBRK	(0x00000001 <<  6)
#define VI_CLR_STATUS_LINE_THRESH	(0x00000001 <<  5)
#define VI_CLR_STATUS_AUX_WRAP		(0x00000001 <<  4)
#define VI_CLR_STATUS_AUX_START_IN	(0x00000001 <<  3)
#define VI_CLR_STATUS_AUX_END_OUT	(0x00000001 <<  2)
#define VI_CLR_STATUS_VID_START_IN	(0x00000001 <<  1)
#define VI_CLR_STATUS_VID_END_OUT	(0x00000001 <<  0)

#define INT_SET_STATUS			0xfec
#define VI_SET_STATUS_AUX_OVRFLW	(0x00000001 <<  9)
#define VI_SET_STATUS_VID_OVRFLW	(0x00000001 <<  8)
#define VI_SET_STATUS_WIN_SEQBRK	(0x00000001 <<  7)
#define VI_SET_STATUS_FID_SEQBRK	(0x00000001 <<  6)
#define VI_SET_STATUS_LINE_THRESH	(0x00000001 <<  5)
#define VI_SET_STATUS_AUX_WRAP		(0x00000001 <<  4)
#define VI_SET_STATUS_AUX_START_IN	(0x00000001 <<  3)
#define VI_SET_STATUS_AUX_END_OUT	(0x00000001 <<  2)
#define VI_SET_STATUS_VID_START_IN	(0x00000001 <<  1)
#define VI_SET_STATUS_VID_END_OUT	(0x00000001 <<  0)

#define VIP_POWER_DOWN			0xff4
#define VI_PWR_DWN			(0x00000001 << 31)




/* -------------- FGPI Registers -------------- */

#define FGPI0				0x00002000
#define FGPI1				0x00003000
#define FGPI2				0x00004000
#define FGPI3				0x00005000

#define FGPI_CONTROL			0x000
#define FGPI_CAPTURE_ENABLE_2		(0x00000001 << 13)
#define FGPI_CAPTURE_ENABLE_1		(0x00000001 << 12)
#define FGPI_MODE			(0x00000001 << 11)
#define FGPI_SAMPLE_SIZE		(0x00000003 <<  8)
#define FGPI_BUF_SYNC_MSG_STOP		(0x00000003 <<  5)
#define FGPI_REC_START_MSG_START	(0x00000003 <<  2)
#define FGPI_TSTAMP_SELECT		(0x00000001 <<  1)
#define FGPI_VAR_LENGTH			(0x00000001 <<  0)

#define FGPI_BASE_1			0x004
#define FGPI_BASE_2			0x008
#define FGPI_SIZE			0x00c
#define FGPI_REC_SIZE			0x010
#define FGPI_STRIDE			0x014
#define FGPI_NUM_RECORD_1		0x018
#define FGPI_NUM_RECORD_2		0x01c
#define FGPI_THRESHOLD_1		0x020
#define FGPI_THRESHOLD_2		0x024

#define INT_STATUS			0xfe0
#define FGPI_BUF1_ACTIVE		(0x00000001 <<  7)
#define FGPI_OVERFLOW			(0x00000001 <<  6)
#define FGPI_MBE			(0x00000001 <<  5)
#define FGPI_UNDERRUN			(0x00000001 <<  4)
#define FGPI_THRESH2_REACHED		(0x00000001 <<  3)
#define FGPI_THRESH1_REACHED		(0x00000001 <<  2)
#define FGPI_BUF2_FULL			(0x00000001 <<  1)
#define FGPI_BUF1_FULL			(0x00000001 <<  0)

#define INT_ENABLE			0xfe4
#define FGPI_OVERFLOW_ENA		(0x00000001 <<  6)
#define FGPI_MBE_ENA			(0x00000001 <<  5)
#define FGPI_UNDERRUN_ENA		(0x00000001 <<  4)
#define FGPI_THRESH2_REACHED_ENA	(0x00000001 <<  3)
#define FGPI_THRESH1_REACHED_ENA	(0x00000001 <<  2)
#define FGPI_BUF2_FULL_ENA		(0x00000001 <<  1)
#define FGPI_BUF1_FULL_ENA		(0x00000001 <<  0)

#define INT_CLR_STATUS			0xfe8
#define FGPI_OVERFLOW_ACK		(0x00000001 <<  6)
#define FGPI_MBE_ACK			(0x00000001 <<  5)
#define FGPI_UNDERRUN_ACK		(0x00000001 <<  4)
#define FGPI_THRESH2_REACHED_ACK	(0x00000001 <<  3)
#define FGPI_THRESH1_REACHED_ACK	(0x00000001 <<  2)
#define FGPI_BUF2_DONE_ACK		(0x00000001 <<  1)
#define FGPI_BUF1_DONE_ACK		(0x00000001 <<  0)

#define INT_SET_STATUS			0xfec
#define FGPI_OVERFLOW_SET		(0x00000001 <<  6)
#define FGPI_MBE_SET			(0x00000001 <<  5)
#define FGPI_UNDERRUN_SET		(0x00000001 <<  4)
#define FGPI_THRESH2_REACHED_SET	(0x00000001 <<  3)
#define FGPI_THRESH1_REACHED_SET	(0x00000001 <<  2)
#define FGPI_BUF2_DONE_SET		(0x00000001 <<  1)
#define FGPI_BUF1_DONE_SET		(0x00000001 <<  0)

#define FGPI_SOFT_RESET			0xff0
#define FGPI_SOFTWARE_RESET		(0x00000001 <<  0)

#define FGPI_INTERFACE			0xff4
#define FGPI_DISABLE_BUS_IF		(0x00000001 <<  0)




/* -------------- AI Registers ---------------- */

#define AI0				0x00006000
#define AI1				0x00007000

#define AI_STATUS			0x000
#define AI_BUF1_ACTIVE			(0x00000001 <<  4)
#define AI_OVERRUN			(0x00000001 <<  3)
#define AI_HBE				(0x00000001 <<  2)
#define AI_BUF2_FULL			(0x00000001 <<  1)
#define AI_BUF1_FULL			(0x00000001 <<  0)

#define AI_CTL				0x004
#define AI_RESET			(0x00000001 <<  31)
#define AI_CAP_ENABLE			(0x00000001 <<  30)
#define AI_CAP_MODE			(0x00000003 <<  28)
#define AI_SIGN_CONVERT			(0x00000001 <<  27)
#define AI_EARLYMODE			(0x00000001 <<  26)
#define AI_DIAGMODE			(0x00000001 <<  25)
#define AI_RAWMODE			(0x00000001 <<  24)
#define AI_OVR_INTEN			(0x00000001 <<   7)
#define AI_HBE_INTEN			(0x00000001 <<   6)
#define AI_BUF2_INTEN			(0x00000001 <<   5)
#define AI_BUF1_INTEN			(0x00000001 <<   4)
#define AI_ACK_OVR			(0x00000001 <<   3)
#define AI_ACK_HBE			(0x00000001 <<   2)
#define AI_ACK2				(0x00000001 <<   1)
#define AI_ACK1				(0x00000001 <<   0)

#define AI_SERIAL			0x008
#define AI_SER_MASTER			(0x00000001 <<  31)
#define AI_DATAMODE			(0x00000001 <<  30)
#define AI_FRAMEMODE			(0x00000003 <<  28)
#define AI_CLOCK_EDGE			(0x00000001 <<  27)
#define AI_SSPOS4			(0x00000001 <<  19)
#define AI_NR_CHAN			(0x00000003 <<  17)
#define AI_WSDIV			(0x000001ff <<   8)
#define AI_SCKDIV			(0x000000ff <<   0)

#define AI_FRAMING			0x00c
#define AI_VALIDPOS			(0x000001ff << 22)
#define AI_LEFTPOS			(0x000001ff << 13)
#define AI_RIGHTPOS			(0x000001ff <<  4)
#define AI_SSPOS_3_0			(0x0000000f <<  0)

#define AI_BASE1			0x014
#define AI_BASE2			0x018
#define AI_BASE				(0x03ffffff <<  6)

#define AI_SIZE				0x01c
#define AI_SAMPLE_SIZE			(0x03ffffff <<  6)

#define AI_INT_ACK			0x020
#define AI_ACK_OVR			(0x00000001 <<  3)
#define AI_ACK_HBE			(0x00000001 <<  2)
#define AI_ACK2				(0x00000001 <<  1)
#define AI_ACK1				(0x00000001 <<  0)

#define AI_PWR_DOWN			0xff4
#define AI_PWR_DWN			(0x00000001 <<  0)

/* -------------- BAM Registers -------------- */

#define BAM				0x00008000

#define BAM_VI0_0_DMA_BUF_MODE		0x000

#define BAM_VI0_0_ADDR_OFFST_0		0x004
#define BAM_VI0_0_ADDR_OFFST_1		0x008
#define BAM_VI0_0_ADDR_OFFST_2		0x00c
#define BAM_VI0_0_ADDR_OFFST_3		0x010
#define BAM_VI0_0_ADDR_OFFST_4		0x014
#define BAM_VI0_0_ADDR_OFFST_5		0x018
#define BAM_VI0_0_ADDR_OFFST_6		0x01c
#define BAM_VI0_0_ADDR_OFFST_7		0x020

#define BAM_VI0_1_DMA_BUF_MODE		0x024
#define BAM_VI0_1_ADDR_OFFST_0		0x028
#define BAM_VI0_1_ADDR_OFFST_1		0x02c
#define BAM_VI0_1_ADDR_OFFST_2		0x030
#define BAM_VI0_1_ADDR_OFFST_3		0x034
#define BAM_VI0_1_ADDR_OFFST_4		0x038
#define BAM_VI0_1_ADDR_OFFST_5		0x03c
#define BAM_VI0_1_ADDR_OFFST_6		0x040
#define BAM_VI0_1_ADDR_OFFST_7		0x044

#define BAM_VI0_2_DMA_BUF_MODE		0x048
#define BAM_VI0_2_ADDR_OFFST_0		0x04c
#define BAM_VI0_2_ADDR_OFFST_1		0x050
#define BAM_VI0_2_ADDR_OFFST_2		0x054
#define BAM_VI0_2_ADDR_OFFST_3		0x058
#define BAM_VI0_2_ADDR_OFFST_4		0x05c
#define BAM_VI0_2_ADDR_OFFST_5		0x060
#define BAM_VI0_2_ADDR_OFFST_6		0x064
#define BAM_VI0_2_ADDR_OFFST_7		0x068


#define BAM_VI1_0_DMA_BUF_MODE		0x06c
#define BAM_VI1_0_ADDR_OFFST_0		0x070
#define BAM_VI1_0_ADDR_OFFST_1		0x074
#define BAM_VI1_0_ADDR_OFFST_2		0x078
#define BAM_VI1_0_ADDR_OFFST_3		0x07c
#define BAM_VI1_0_ADDR_OFFST_4		0x080
#define BAM_VI1_0_ADDR_OFFST_5		0x084
#define BAM_VI1_0_ADDR_OFFST_6		0x088
#define BAM_VI1_0_ADDR_OFFST_7		0x08c

#define BAM_VI1_1_DMA_BUF_MODE		0x090
#define BAM_VI1_1_ADDR_OFFST_0		0x094
#define BAM_VI1_1_ADDR_OFFST_1		0x098
#define BAM_VI1_1_ADDR_OFFST_2		0x09c
#define BAM_VI1_1_ADDR_OFFST_3		0x0a0
#define BAM_VI1_1_ADDR_OFFST_4		0x0a4
#define BAM_VI1_1_ADDR_OFFST_5		0x0a8
#define BAM_VI1_1_ADDR_OFFST_6		0x0ac
#define BAM_VI1_1_ADDR_OFFST_7		0x0b0

#define BAM_VI1_2_DMA_BUF_MODE		0x0b4
#define BAM_VI1_2_ADDR_OFFST_0		0x0b8
#define BAM_VI1_2_ADDR_OFFST_1		0x0bc
#define BAM_VI1_2_ADDR_OFFST_2		0x0c0
#define BAM_VI1_2_ADDR_OFFST_3		0x0c4
#define BAM_VI1_2_ADDR_OFFST_4		0x0c8
#define BAM_VI1_2_ADDR_OFFST_5		0x0cc
#define BAM_VI1_2_ADDR_OFFST_6		0x0d0
#define BAM_VI1_2_ADDR_OFFST_7		0x0d4


#define BAM_FGPI0_DMA_BUF_MODE		0x0d8
#define BAM_FGPI0_ADDR_OFFST_0		0x0dc
#define BAM_FGPI0_ADDR_OFFST_1		0x0e0
#define BAM_FGPI0_ADDR_OFFST_2		0x0e4
#define BAM_FGPI0_ADDR_OFFST_3		0x0e8
#define BAM_FGPI0_ADDR_OFFST_4		0x0ec
#define BAM_FGPI0_ADDR_OFFST_5		0x0f0
#define BAM_FGPI0_ADDR_OFFST_6		0x0f4
#define BAM_FGPI0_ADDR_OFFST_7		0x0f8

#define BAM_FGPI1_DMA_BUF_MODE		0x0fc
#define BAM_FGPI1_ADDR_OFFST_0		0x100
#define BAM_FGPI1_ADDR_OFFST_1		0x104
#define BAM_FGPI1_ADDR_OFFST_2		0x108
#define BAM_FGPI1_ADDR_OFFST_3		0x10c
#define BAM_FGPI1_ADDR_OFFST_4		0x110
#define BAM_FGPI1_ADDR_OFFST_5		0x114
#define BAM_FGPI1_ADDR_OFFST_6		0x118
#define BAM_FGPI1_ADDR_OFFST_7		0x11c

#define BAM_FGPI2_DMA_BUF_MODE		0x120
#define BAM_FGPI2_ADDR_OFFST_0		0x124
#define BAM_FGPI2_ADDR_OFFST_1		0x128
#define BAM_FGPI2_ADDR_OFFST_2		0x12c
#define BAM_FGPI2_ADDR_OFFST_3		0x130
#define BAM_FGPI2_ADDR_OFFST_4		0x134
#define BAM_FGPI2_ADDR_OFFST_5		0x138
#define BAM_FGPI2_ADDR_OFFST_6		0x13c
#define BAM_FGPI2_ADDR_OFFST_7		0x140

#define BAM_FGPI3_DMA_BUF_MODE		0x144
#define BAM_FGPI3_ADDR_OFFST_0		0x148
#define BAM_FGPI3_ADDR_OFFST_1		0x14c
#define BAM_FGPI3_ADDR_OFFST_2		0x150
#define BAM_FGPI3_ADDR_OFFST_3		0x154
#define BAM_FGPI3_ADDR_OFFST_4		0x158
#define BAM_FGPI3_ADDR_OFFST_5		0x15c
#define BAM_FGPI3_ADDR_OFFST_6		0x160
#define BAM_FGPI3_ADDR_OFFST_7		0x164


#define BAM_AI0_DMA_BUF_MODE		0x168
#define BAM_AI0_ADDR_OFFST_0		0x16c
#define BAM_AI0_ADDR_OFFST_1		0x170
#define BAM_AI0_ADDR_OFFST_2		0x174
#define BAM_AI0_ADDR_OFFST_3		0x178
#define BAM_AI0_ADDR_OFFST_4		0x17c
#define BAM_AIO_ADDR_OFFST_5		0x180
#define BAM_AI0_ADDR_OFFST_6		0x184
#define BAM_AIO_ADDR_OFFST_7		0x188

#define BAM_AI1_DMA_BUF_MODE		0x18c
#define BAM_AI1_ADDR_OFFST_0		0x190
#define BAM_AI1_ADDR_OFFST_1		0x194
#define BAM_AI1_ADDR_OFFST_2		0x198
#define BAM_AI1_ADDR_OFFST_3		0x19c
#define BAM_AI1_ADDR_OFFST_4		0x1a0
#define BAM_AI1_ADDR_OFFST_5		0x1a4
#define BAM_AI1_ADDR_OFFST_6		0x1a8
#define BAM_AI1_ADDR_OFFST_7		0x1ac

#define BAM_SW_RST			0xff0
#define BAM_SW_RESET			(0x00000001 <<  0)





/* -------------- MMU Registers -------------- */

#define MMU				0x00009000

#define MMU_MODE			0x000

#define MMU_DMA_CONFIG0			0x004
#define MMU_DMA_CONFIG1			0x008
#define MMU_DMA_CONFIG2			0x00c
#define MMU_DMA_CONFIG3			0x010
#define MMU_DMA_CONFIG4			0x014
#define MMU_DMA_CONFIG5			0x018
#define MMU_DMA_CONFIG6			0x01c
#define MMU_DMA_CONFIG7			0x020
#define MMU_DMA_CONFIG8			0x024
#define MMU_DMA_CONFIG9			0x028
#define MMU_DMA_CONFIG10		0x02c
#define MMU_DMA_CONFIG11		0x030
#define MMU_DMA_CONFIG12		0x034
#define MMU_DMA_CONFIG13		0x038
#define MMU_DMA_CONFIG14		0x03c
#define MMU_DMA_CONFIG15		0x040

#define MMU_SW_RST			0xff0
#define MMU_SW_RESET			(0x0001 <<  0)

#define MMU_PTA_BASE0			0x044 /* DMA 0 */
#define MMU_PTA_BASE1			0x084 /* DMA 1 */
#define MMU_PTA_BASE2			0x0c4 /* DMA 2 */
#define MMU_PTA_BASE3			0x104 /* DMA 3 */
#define MMU_PTA_BASE4			0x144 /* DMA 4 */
#define MMU_PTA_BASE5			0x184 /* DMA 5 */
#define MMU_PTA_BASE6			0x1c4 /* DMA 6 */
#define MMU_PTA_BASE7			0x204 /* DMA 7 */
#define MMU_PTA_BASE8			0x244 /* DMA 8 */
#define MMU_PTA_BASE9			0x284 /* DMA 9 */
#define MMU_PTA_BASE10			0x2c4 /* DMA 10 */
#define MMU_PTA_BASE11			0x304 /* DMA 11 */
#define MMU_PTA_BASE12			0x344 /* DMA 12 */
#define MMU_PTA_BASE13			0x384 /* DMA 13 */
#define MMU_PTA_BASE14			0x3c4 /* DMA 14 */
#define MMU_PTA_BASE15			0x404 /* DMA 15 */

#define MMU_PTA0_LSB			MMU_PTA_BASE + 0x00
#define MMU_PTA0_MSB			MMU_PTA_BASE + 0x04
#define MMU_PTA1_LSB			MMU_PTA_BASE + 0x08
#define MMU_PTA1_MSB			MMU_PTA_BASE + 0x0c
#define MMU_PTA2_LSB			MMU_PTA_BASE + 0x10
#define MMU_PTA2_MSB			MMU_PTA_BASE + 0x14
#define MMU_PTA3_LSB			MMU_PTA_BASE + 0x18
#define MMU_PTA3_MSB			MMU_PTA_BASE + 0x1c
#define MMU_PTA4_LSB			MMU_PTA_BASE + 0x20
#define MMU_PTA4_MSB			MMU_PTA_BASE + 0x24
#define MMU_PTA5_LSB			MMU_PTA_BASE + 0x28
#define MMU_PTA5_MSB			MMU_PTA_BASE + 0x2c
#define MMU_PTA6_LSB			MMU_PTA_BASE + 0x30
#define MMU_PTA6_MSB			MMU_PTA_BASE + 0x34
#define MMU_PTA7_LSB			MMU_PTA_BASE + 0x38
#define MMU_PTA7_MSB			MMU_PTA_BASE + 0x3c





/* -------------- MSI Registers -------------- */

#define MSI				0x0000a000

#define MSI_DELAY_TIMER			0x000
#define MSI_DELAY_1CLK			(0x00000001 <<  0)
#define MSI_DELAY_2CLK			(0x00000002 <<  0)

#define MSI_INTA_POLARITY		0x004
#define MSI_INTA_POLARITY_HIGH		(0x00000001 <<  0)

#define MSI_CONFIG0			0x008
#define MSI_CONFIG1			0x00c
#define MSI_CONFIG2			0x010
#define MSI_CONFIG3			0x014
#define MSI_CONFIG4			0x018
#define MSI_CONFIG5			0x01c
#define MSI_CONFIG6			0x020
#define MSI_CONFIG7			0x024
#define MSI_CONFIG8			0x028
#define MSI_CONFIG9			0x02c
#define MSI_CONFIG10			0x030
#define MSI_CONFIG11			0x034
#define MSI_CONFIG12			0x038
#define MSI_CONFIG13			0x03c
#define MSI_CONFIG14			0x040
#define MSI_CONFIG15			0x044
#define MSI_CONFIG16			0x048
#define MSI_CONFIG17			0x04c
#define MSI_CONFIG18			0x050
#define MSI_CONFIG19			0x054
#define MSI_CONFIG20			0x058
#define MSI_CONFIG21			0x05c
#define MSI_CONFIG22			0x060
#define MSI_CONFIG23			0x064
#define MSI_CONFIG24			0x068
#define MSI_CONFIG25			0x06c
#define MSI_CONFIG26			0x070
#define MSI_CONFIG27			0x074
#define MSI_CONFIG28			0x078
#define MSI_CONFIG29			0x07c
#define MSI_CONFIG30			0x080
#define MSI_CONFIG31			0x084
#define MSI_CONFIG32			0x088
#define MSI_CONFIG33			0x08c
#define MSI_CONFIG34			0x090
#define MSI_CONFIG35			0x094
#define MSI_CONFIG36			0x098
#define MSI_CONFIG37			0x09c
#define MSI_CONFIG38			0x0a0
#define MSI_CONFIG39			0x0a4
#define MSI_CONFIG40			0x0a8
#define MSI_CONFIG41			0x0ac
#define MSI_CONFIG42			0x0b0
#define MSI_CONFIG43			0x0b4
#define MSI_CONFIG44			0x0b8
#define MSI_CONFIG45			0x0bc
#define MSI_CONFIG46			0x0c0
#define MSI_CONFIG47			0x0c4
#define MSI_CONFIG48			0x0c8
#define MSI_INT_POL_EDGE_RISE		(0x00000001 << 24)
#define MSI_INT_POL_EDGE_FALL		(0x00000002 << 24)
#define MSI_INT_POL_EDGE_ANY		(0x00000003 << 24)
#define MSI_TC				(0x00000007 << 16)
#define MSI_ID				(0x0000000f <<  0)

#define MSI_INT_STATUS_L		0xfc0
#define MSI_INT_TAGACK_VI0_0		(0x00000001 <<  0)
#define MSI_INT_TAGACK_VI0_1		(0x00000001 <<  1)
#define MSI_INT_TAGACK_VI0_2		(0x00000001 <<  2)
#define MSI_INT_TAGACK_VI1_0		(0x00000001 <<  3)
#define MSI_INT_TAGACK_VI1_1		(0x00000001 <<  4)
#define MSI_INT_TAGACK_VI1_2		(0x00000001 <<  5)
#define MSI_INT_TAGACK_FGPI_0		(0x00000001 <<  6)
#define MSI_INT_TAGACK_FGPI_1		(0x00000001 <<  7)
#define MSI_INT_TAGACK_FGPI_2		(0x00000001 <<  8)
#define MSI_INT_TAGACK_FGPI_3		(0x00000001 <<  9)
#define MSI_INT_TAGACK_AI_0		(0x00000001 << 10)
#define MSI_INT_TAGACK_AI_1		(0x00000001 << 11)
#define MSI_INT_OVRFLW_VI0_0		(0x00000001 << 12)
#define MSI_INT_OVRFLW_VI0_1		(0x00000001 << 13)
#define MSI_INT_OVRFLW_VI0_2		(0x00000001 << 14)
#define MSI_INT_OVRFLW_VI1_0		(0x00000001 << 15)
#define MSI_INT_OVRFLW_VI1_1		(0x00000001 << 16)
#define MSI_INT_OVRFLW_VI1_2		(0x00000001 << 17)
#define MSI_INT_OVRFLW_FGPI_O		(0x00000001 << 18)
#define MSI_INT_OVRFLW_FGPI_1		(0x00000001 << 19)
#define MSI_INT_OVRFLW_FGPI_2		(0x00000001 << 20)
#define MSI_INT_OVRFLW_FGPI_3		(0x00000001 << 21)
#define MSI_INT_OVRFLW_AI_0		(0x00000001 << 22)
#define MSI_INT_OVRFLW_AI_1		(0x00000001 << 23)
#define MSI_INT_AVINT_VI0		(0x00000001 << 24)
#define MSI_INT_AVINT_VI1		(0x00000001 << 25)
#define MSI_INT_AVINT_FGPI_0		(0x00000001 << 26)
#define MSI_INT_AVINT_FGPI_1		(0x00000001 << 27)
#define MSI_INT_AVINT_FGPI_2		(0x00000001 << 28)
#define MSI_INT_AVINT_FGPI_3		(0x00000001 << 29)
#define MSI_INT_AVINT_AI_0		(0x00000001 << 30)
#define MSI_INT_AVINT_AI_1		(0x00000001 << 31)

#define MSI_INT_STATUS_H		0xfc4
#define MSI_INT_UNMAPD_TC_INT		(0x00000001 <<  0)
#define MSI_INT_EXTINT_0		(0x00000001 <<  1)
#define MSI_INT_EXTINT_1		(0x00000001 <<  2)
#define MSI_INT_EXTINT_2		(0x00000001 <<  3)
#define MSI_INT_EXTINT_3		(0x00000001 <<  4)
#define MSI_INT_EXTINT_4		(0x00000001 <<  5)
#define MSI_INT_EXTINT_5		(0x00000001 <<  6)
#define MSI_INT_EXTINT_6		(0x00000001 <<  7)
#define MSI_INT_EXTINT_7		(0x00000001 <<  8)
#define MSI_INT_EXTINT_8		(0x00000001 <<  9)
#define MSI_INT_EXTINT_9		(0x00000001 << 10)
#define MSI_INT_EXTINT_10		(0x00000001 << 11)
#define MSI_INT_EXTINT_11		(0x00000001 << 12)
#define MSI_INT_EXTINT_12		(0x00000001 << 13)
#define MSI_INT_EXTINT_13		(0x00000001 << 14)
#define MSI_INT_EXTINT_14		(0x00000001 << 15)
#define MSI_INT_EXTINT_15		(0x00000001 << 16)
#define MSI_INT_I2CINT_0		(0x00000001 << 17)
#define MSI_INT_I2CINT_1		(0x00000001 << 18)

#define MSI_INT_STATUS_CLR_L		0xfc8
#define MSI_INT_STATUS_CLR_H		0xfcc
#define MSI_INT_STATUS_SET_L		0xfd0
#define MSI_INT_STATUS_SET_H		0xfd4
#define MSI_INT_ENA_L			0xfd8
#define MSI_INT_ENA_H			0xfdc
#define MSI_INT_ENA_CLR_L		0xfe0
#define MSI_INT_ENA_CLR_H		0xfe4
#define MSI_INT_ENA_SET_L		0xfe8
#define MSI_INT_ENA_SET_H		0xfec

#define MSI_SW_RST			0xff0
#define MSI_SW_RESET			(0x0001 <<  0)




/* -------------- I2C Registers -------------- */

#define I2C_B				0x0000b000
#define I2C_A				0x0000c000

#define RX_FIFO				0x000
#define I2C_RX_BYTE			(0x000000ff <<  0)

#define TX_FIFO				0x000
#define I2C_STOP_BIT			(0x00000001 <<  9)
#define I2C_START_BIT			(0x00000001 <<  8)
#define I2C_TX_BYTE			(0x000000ff <<  0)

#define I2C_STATUS			0x008
#define I2C_TRANSMIT			(0x00000001 << 11)
#define I2C_RECEIVE			(0x00000001 << 10)
#define I2C_TRANSMIT_S_PROG		(0x00000001 <<  9)
#define I2C_TRANSMIT_S_CLEAR		(0x00000001 <<  8)
#define I2C_TRANSMIT_PROG		(0x00000001 <<  7)
#define I2C_TRANSMIT_CLEAR		(0x00000001 <<  6)
#define I2C_RECEIVE_PROG		(0x00000001 <<  5)
#define I2C_RECEIVE_CLEAR		(0x00000001 <<  4)
#define I2C_SDA_LINE			(0x00000001 <<  3)
#define I2C_SCL_LINE			(0x00000001 <<  2)
#define I2C_START_STOP_FLAG		(0x00000001 <<  1)
#define I2C_MODE_STATUS			(0x00000001 <<  0)

#define I2C_CONTROL			0x00c
#define I2C_SCL_CONTROL			(0x00000001 <<  7)
#define I2C_SDA_CONTROL			(0x00000001 <<  6)
#define I2C_RECEIVE_PROTECT		(0x00000001 <<  5)
#define I2C_RECEIVE_PRO_READ		(0x00000001 <<  4)
#define I2C_TRANS_SELF_CLEAR		(0x00000001 <<  3)
#define I2C_TRANS_S_SELF_CLEAR		(0x00000001 <<  2)
#define I2C_SLAVE_ADDR_10BIT		(0x00000001 <<  1)
#define I2C_RESET			(0x00000001 <<  0)

#define I2C_CLOCK_DIVISOR_HIGH		0x010
#define I2C_CLOCK_HIGH			(0x0000ffff <<  0)

#define I2C_CLOCK_DIVISOR_LOW		0x014
#define I2C_CLOCK_LOW			(0x0000ffff <<  0)

#define I2C_RX_LEVEL			0x01c
#define I2C_RECEIVE_RANGE		(0x0000007f <<  0)

#define I2C_TX_LEVEL			0x020
#define I2C_TRANSMIT_RANGE		(0x0000007f <<  0)

#define I2C_SDA_HOLD			0x028
#define I2C_HOLD_TIME			(0x0000007f <<  0)

#define MODULE_CONF			0xfd4
#define INT_CLR_ENABLE			0xfd8
#define I2C_CLR_ENABLE_STFNF		(0x00000001 << 12)
#define I2C_CLR_ENABLE_MTFNF		(0x00000001 << 11)
#define I2C_CLR_ENABLE_RFDA		(0x00000001 << 10)
#define I2C_CLR_ENABLE_RFF		(0x00000001 <<  9)
#define I2C_CLR_ENABLE_STDR		(0x00000001 <<  8)
#define I2C_CLR_ENABLE_MTDR		(0x00000001 <<  7)
#define I2C_CLR_ENABLE_IBE		(0x00000001 <<  6)
#define I2C_CLR_ENABLE_MSMC		(0x00000001 <<  5)
#define I2C_CLR_ENABLE_SRSD		(0x00000001 <<  4)
#define I2C_CLR_ENABLE_STSD		(0x00000001 <<  3)
#define I2C_CLR_ENABLE_MTNA		(0x00000001 <<  2)
#define I2C_CLR_ENABLE_MAF		(0x00000001 <<  1)
#define I2C_CLR_ENABLE_MTD		(0x00000001 <<  0)

#define INT_SET_ENABLE			0xfdc
#define I2C_SET_ENABLE_STFNF		(0x00000001 << 12)
#define I2C_SET_ENABLE_MTFNF		(0x00000001 << 11)
#define I2C_SET_ENABLE_RFDA		(0x00000001 << 10)
#define I2C_SET_ENABLE_RFF		(0x00000001 <<  9)
#define I2C_SET_ENABLE_STDR		(0x00000001 <<  8)
#define I2C_SET_ENABLE_MTDR		(0x00000001 <<  7)
#define I2C_SET_ENABLE_IBE		(0x00000001 <<  6)
#define I2C_SET_ENABLE_MSMC		(0x00000001 <<  5)
#define I2C_SET_ENABLE_SRSD		(0x00000001 <<  4)
#define I2C_SET_ENABLE_STSD		(0x00000001 <<  3)
#define I2C_SET_ENABLE_MTNA		(0x00000001 <<  2)
#define I2C_SET_ENABLE_MAF		(0x00000001 <<  1)
#define I2C_SET_ENABLE_MTD		(0x00000001 <<  0)

#define INT_STATUS			0xfe0
#define I2C_INTERRUPT_STFNF		(0x00000001 << 12)
#define I2C_INTERRUPT_MTFNF		(0x00000001 << 11)
#define I2C_INTERRUPT_RFDA		(0x00000001 << 10)
#define I2C_INTERRUPTE_RFF		(0x00000001 <<  9)
#define I2C_SLAVE_INTERRUPT_STDR	(0x00000001 <<  8)
#define I2C_MASTER_INTERRUPT_MTDR	(0x00000001 <<  7)
#define I2C_ERROR_IBE			(0x00000001 <<  6)
#define I2C_MODE_CHANGE_INTER_MSMC	(0x00000001 <<  5)
#define I2C_SLAVE_RECEIVE_INTER_SRSD	(0x00000001 <<  4)
#define I2C_SLAVE_TRANSMIT_INTER_STSD	(0x00000001 <<  3)
#define I2C_ACK_INTER_MTNA		(0x00000001 <<  2)
#define I2C_FAILURE_INTER_MAF		(0x00000001 <<  1)
#define I2C_INTERRUPT_MTD		(0x00000001 <<  0)

#define INT_ENABLE			0xfe4
#define I2C_ENABLE_STFNF		(0x00000001 << 12)
#define I2C_ENABLE_MTFNF		(0x00000001 << 11)
#define I2C_ENABLE_RFDA			(0x00000001 << 10)
#define I2C_ENABLE_RFF			(0x00000001 <<  9)
#define I2C_ENABLE_STDR			(0x00000001 <<  8)
#define I2C_ENABLE_MTDR			(0x00000001 <<  7)
#define I2C_ENABLE_IBE			(0x00000001 <<  6)
#define I2C_ENABLE_MSMC			(0x00000001 <<  5)
#define I2C_ENABLE_SRSD			(0x00000001 <<  4)
#define I2C_ENABLE_STSD			(0x00000001 <<  3)
#define I2C_ENABLE_MTNA			(0x00000001 <<  2)
#define I2C_ENABLE_MAF			(0x00000001 <<  1)
#define I2C_ENABLE_MTD			(0x00000001 <<  0)

#define INT_CLR_STATUS			0xfe8
#define I2C_CLR_STATUS_STFNF		(0x00000001 << 12)
#define I2C_CLR_STATUS_MTFNF		(0x00000001 << 11)
#define I2C_CLR_STATUS_RFDA		(0x00000001 << 10)
#define I2C_CLR_STATUS_RFF		(0x00000001 <<  9)
#define I2C_CLR_STATUS_STDR		(0x00000001 <<  8)
#define I2C_CLR_STATUS_MTDR		(0x00000001 <<  7)
#define I2C_CLR_STATUS_IBE		(0x00000001 <<  6)
#define I2C_CLR_STATUS_MSMC		(0x00000001 <<  5)
#define I2C_CLR_STATUS_SRSD		(0x00000001 <<  4)
#define I2C_CLR_STATUS_STSD		(0x00000001 <<  3)
#define I2C_CLR_STATUS_MTNA		(0x00000001 <<  2)
#define I2C_CLR_STATUS_MAF		(0x00000001 <<  1)
#define I2C_CLR_STATIS_MTD		(0x00000001 <<  0)

#define INT_SET_STATUS			0xfec
#define I2C_SET_STATUS_STFNF		(0x00000001 << 12)
#define I2C_SET_STATUS_MTFNF		(0x00000001 << 11)
#define I2C_SET_STATUS_RFDA		(0x00000001 << 10)
#define I2C_SET_STATUS_RFF		(0x00000001 <<  9)
#define I2C_SET_STATUS_STDR		(0x00000001 <<  8)
#define I2C_SET_STATUS_MTDR		(0x00000001 <<  7)
#define I2C_SET_STATUS_IBE		(0x00000001 <<  6)
#define I2C_SET_STATUS_MSMC		(0x00000001 <<  5)
#define I2C_SET_STATUS_SRSD		(0x00000001 <<  4)
#define I2C_SET_STATUS_STSD		(0x00000001 <<  3)
#define I2C_SET_STATUS_MTNA		(0x00000001 <<  2)
#define I2C_SET_STATUS_MAF		(0x00000001 <<  1)
#define I2C_SET_STATIS_MTD		(0x00000001 <<  0)




/* -------------- SPI Registers -------------- */

#define SPI_CONTROL_REG			0x000
#define SPI_SERIAL_INTER_ENABLE		(0x00000001 <<  7)
#define SPI_LSB_FIRST_ENABLE		(0x00000001 <<  6)
#define SPI_MODE_SELECT			(0x00000001 <<  5)
#define SPI_CLOCK_POLARITY		(0x00000001 <<  4)
#define SPI_CLOCK_PHASE			(0x00000001 <<  3)

#define SPI_STATUS			0x004
#define SPI_TRANSFER_FLAG		(0x00000001 <<  7)
#define SPI_WRITE_COLLISSION		(0x00000001 <<  6)
#define SPI_READ_OVERRUN		(0x00000001 <<  5)
#define SPI_MODE_FAULT			(0x00000001 <<  4)
#define SPI_SLAVE_ABORT			(0x00000001 <<  3)

#define SPI_DATA			0x008
#define SPI_BIDI_DATA			(0x000000ff <<  0)

#define SPI_CLOCK_COUNTER		0x00c
#define SPI_CLOCK			(0x00000001 <<  0)




/* -------------- GPIO Registers -------------- */

#define GPIO				0x0000e000

#define GPIO_RD				0x000
#define GPIO_WR				0x004
#define GPIO_WR_MODE			0x008
#define GPIO_OEN			0x00c

#define GPIO_SW_RST			0xff0
#define GPIO_SW_RESET			(0x00000001 <<  0)




/* -------------- PHI0 Registers -------------- */





/* -------------- CGU Registers -------------- */

#define CGU				0x00013000

#define CGU_SCR_0			0x000
#define CGU_SCR_1			0x004
#define CGU_SCR_2			0x008
#define CGU_SCR_3			0x00c
#define CGU_SCR_4			0x010
#define CGU_SCR_5			0x014
#define CGU_SCR_6			0x018
#define CGU_SCR_7			0x01c
#define CGU_SCR_8			0x020
#define CGU_SCR_9			0x024
#define CGU_SCR_10			0x028
#define CGU_SCR_11			0x02c
#define CGU_SCR_12			0x030
#define CGU_SCR_13			0x034
#define CGU_SCR_STOP			(0x00000001 <<  3)
#define CGU_SCR_RESET			(0x00000001 <<  2)
#define CGU_SCR_ENF2			(0x00000001 <<  1)
#define CGU_SCR_ENF1			(0x00000001 <<  0)

#define CGU_FS1_0			0x038
#define CGU_FS1_1			0x03c
#define CGU_FS1_2			0x040
#define CGU_FS1_3			0x044
#define CGU_FS1_4			0x048
#define CGU_FS1_5			0x04c
#define CGU_FS1_6			0x050
#define CGU_FS1_7			0x054
#define CGU_FS1_8			0x058
#define CGU_FS1_9			0x05c
#define CGU_FS1_10			0x060
#define CGU_FS1_11			0x064
#define CGU_FS1_12			0x068
#define CGU_FS1_13			0x06c

#define CGU_FS2_0			0x070
#define CGU_FS2_1			0x074
#define CGU_FS2_2			0x078
#define CGU_FS2_3			0x07c
#define CGU_FS2_4			0x080
#define CGU_FS2_5			0x084
#define CGU_FS2_6			0x088
#define CGU_FS2_7			0x08c
#define CGU_FS2_8			0x090
#define CGU_FS2_9			0x094
#define CGU_FS2_10			0x098
#define CGU_FS2_11			0x09c
#define CGU_FS2_12			0x0a0
#define CGU_FS2_13			0x0a4

#define CGU_SSR_0			0x0a8
#define CGU_SSR_1			0x0ac
#define CGU_SSR_2			0x0b0
#define CGU_SSR_3			0x0b4
#define CGU_SSR_4			0x0b8
#define CGU_SSR_5			0x0bc
#define CGU_SSR_6			0x0c0
#define CGU_SSR_7			0x0c4
#define CGU_SSR_8			0x0c8
#define CGU_SSR_9			0x0cc
#define CGU_SSR_10			0x0d0
#define CGU_SSR_11			0x0d4
#define CGU_SSR_12			0x0d8
#define CGU_SSR_13			0x0dc

#define CGU_PCR_0_0			0x0e0
#define CGU_PCR_0_1			0x0e4
#define CGU_PCR_0_2			0x0e8
#define CGU_PCR_0_3			0x0ec
#define CGU_PCR_0_4			0x0f0
#define CGU_PCR_0_5			0x0f4
#define CGU_PCR_0_6			0x0f8
#define CGU_PCR_0_7			0x0fc
#define CGU_PCR_1_0			0x100
#define CGU_PCR_1_1			0x104
#define CGU_PCR_2_0			0x108
#define CGU_PCR_2_1			0x10c
#define CGU_PCR_3_0			0x110
#define CGU_PCR_3_1			0x114
#define CGU_PCR_3_2			0x118
#define CGU_PCR_4_0			0x11c
#define CGU_PCR_4_1			0x120
#define CGU_PCR_5			0x124
#define CGU_PCR_6			0x128
#define CGU_PCR_7			0x12c
#define CGU_PCR_8			0x130
#define CGU_PCR_9			0x134
#define CGU_PCR_10			0x138
#define CGU_PCR_11			0x13c
#define CGU_PCR_12			0x140
#define CGU_PCR_13			0x144
#define CGU_PCR_WAKE_EN			(0x00000001 <<  2)
#define CGU_PCR_AUTO			(0x00000001 <<  1)
#define CGU_PCR_RUN			(0x00000001 <<  0)


#define CGU_PSR_0_0			0x148
#define CGU_PSR_0_1			0x14c
#define CGU_PSR_0_2			0x150
#define CGU_PSR_0_3			0x154
#define CGU_PSR_0_4			0x158
#define CGU_PSR_0_5			0x15c
#define CGU_PSR_0_6			0x160
#define CGU_PSR_0_7			0x164
#define CGU_PSR_1_0			0x168
#define CGU_PSR_1_1			0x16c
#define CGU_PSR_2_0			0x170
#define CGU_PSR_2_1			0x174
#define CGU_PSR_3_0			0x178
#define CGU_PSR_3_1			0x17c
#define CGU_PSR_3_2			0x180
#define CGU_PSR_4_0			0x184
#define CGU_PSR_4_1			0x188
#define CGU_PSR_5			0x18c
#define CGU_PSR_6			0x190
#define CGU_PSR_7			0x194
#define CGU_PSR_8			0x198
#define CGU_PSR_9			0x19c
#define CGU_PSR_10			0x1a0
#define CGU_PSR_11			0x1a4
#define CGU_PSR_12			0x1a8
#define CGU_PSR_13			0x1ac

#define CGU_ESR_0_0			0x1b0
#define CGU_ESR_0_1			0x1b4
#define CGU_ESR_0_2			0x1b8
#define CGU_ESR_0_3			0x1bc
#define CGU_ESR_0_4			0x1c0
#define CGU_ESR_0_5			0x1c4
#define CGU_ESR_0_6			0x1c8
#define CGU_ESR_0_7			0x1cc
#define CGU_ESR_1_0			0x1d0
#define CGU_ESR_1_1			0x1d4
#define CGU_ESR_2_0			0x1d8
#define CGU_ESR_2_1			0x1dc
#define CGU_ESR_2_2			0x1e0
#define CGU_ESR_3_0			0x1e4
#define CGU_ESR_3_1			0x1e8
#define CGU_ESR_4_0			0x1ec
#define CGU_ESR_4_1			0x1f0
#define CGU_ESR_5			0x1f4
#define CGU_ESR_6			0x1f8
#define CGU_ESR_7			0x1fc
#define CGU_ESR_8			0x200
#define CGU_ESR_9			0x204
#define CGU_ESR_10			0x208
#define CGU_ESR_11			0x20c
#define CGU_ESR_12			0x210
#define CGU_ESR_13			0x214
#define CGU_ESR_FD_EN			(0x00000001 <<  0)

#define CGU_FDC_0			0x218
#define CGU_FDC_1			0x21c
#define CGU_FDC_2			0x220
#define CGU_FDC_3			0x224
#define CGU_FDC_4			0x228
#define CGU_FDC_5			0x22c
#define CGU_FDC_6			0x230
#define CGU_FDC_7			0x234
#define CGU_FDC_8			0x238
#define CGU_FDC_9			0x23c
#define CGU_FDC_10			0x240
#define CGU_FDC_11			0x244
#define CGU_FDC_12			0x248
#define CGU_FDC_13			0x24c




/* -------------- DCS Registers -------------- */

#define DCS				0x00014000

#define DCSC_CTRL			0x000
#define DCSC_SEL_PLLDI			(0x03ffffff <<  5)
#define DCSC_TOUT_SEL			(0x0000000f <<  1)
#define DCSC_TOUT_OFF			(0x00000001 <<  0)

#define DCSC_ADDR			0x00c
#define DCSC_ERR_TOUT_ADDR		(0x3fffffff <<  2)

#define DCSC_STAT			0x010
#define DCSC_ERR_TOUT_GNT		(0x0000001f << 24)
#define DCSC_ERR_TOUT_SEL		(0x0000007f << 10)
#define DCSC_ERR_TOUT_READ		(0x00000001 <<  8)
#define DCSC_ERR_TOUT_MASK		(0x0000000f <<  4)
#define DCSC_ERR_ACK			(0x00000001 <<  1)

#define DCSC_FEATURES			0x040
#define DCSC_UNIQUE_ID			(0x00000007 << 16)
#define DCSC_SECURITY			(0x00000001 << 14)
#define DCSC_NUM_BASE_REGS		(0x00000003 << 11)
#define DCSC_NUM_TARGETS		(0x0000001f <<  5)
#define DCSC_NUM_INITIATORS		(0x0000001f <<  0)

#define DCSC_BASE_REG0			0x100
#define DCSC_BASE_N_REG			(0x00000fff << 20)

#define DCSC_INT_CLR_ENABLE		0xfd8
#define DCSC_INT_CLR_ENABLE_TOUT	(0x00000001 <<  1)
#define DCSC_INT_CLR_ENABLE_ERROR	(0x00000001 <<  0)

#define DCSC_INT_SET_ENABLE		0xfdc
#define DCSC_INT_SET_ENABLE_TOUT	(0x00000001 <<  1)
#define DCSC_INT_SET_ENABLE_ERROR	(0x00000001 <<  0)

#define DCSC_INT_STATUS			0xfe0
#define DCSC_INT_STATUS_TOUT		(0x00000001 <<  1)
#define DCSC_INT_STATUS_ERROR		(0x00000001 <<  0)

#define DCSC_INT_ENABLE			0xfe4
#define DCSC_INT_ENABLE_TOUT		(0x00000001 <<  1)
#define DCSC_INT_ENABLE_ERROR		(0x00000001 <<  0)

#define DCSC_INT_CLR_STATUS		0xfe8
#define DCSC_INT_CLEAR_TOUT		(0x00000001 <<  1)
#define DCSC_INT_CLEAR_ERROR		(0x00000001 <<  0)

#define DCSC_INT_SET_STATUS		0xfec
#define DCSC_INT_SET_TOUT		(0x00000001 <<  1)
#define DCSC_INT_SET_ERROR		(0x00000001 <<  0)




/* -------------- GREG Registers -------------- */

#define GREG				0x00012000

#define GREG_SUBSYS_CONFIG		0x000
#define GREG_SUBSYS_ID			(0x0000ffff << 16)
#define GREG_SUBSYS_VID			(0x0000ffff <<  0)

#define GREG_MSI_BAR_PMCSR		0x004
#define GREG_PMCSR_SCALE_7		(0x00000003 << 30)
#define GREG_PMCSR_SCALE_6		(0x00000003 << 28)
#define GREG_PMCSR_SCALE_5		(0x00000003 << 26)
#define GREG_PMCSR_SCALE_4		(0x00000003 << 24)
#define GREG_PMCSR_SCALE_3		(0x00000003 << 22)
#define GREG_PMCSR_SCALE_2		(0x00000003 << 20)
#define GREG_PMCSR_SCALE_1		(0x00000003 << 18)
#define GREG_PMCSR_SCALE_0		(0x00000003 << 16)

#define GREG_BAR_WIDTH_17		(0x0000001e <<  8)
#define GREG_BAR_WIDTH_18		(0x0000001c <<  8)
#define GREG_BAR_WIDTH_19		(0x00000018 <<  8)
#define GREG_BAR_WIDTH_20		(0x00000010 <<  8)

#define GREG_BAR_PREFETCH		(0x00000001 <<  3)
#define GREG_MSI_MM_CAP1		(0x00000000 <<  0) // FIXME !
#define GREG_MSI_MM_CAP2		(0x00000001 <<  0)
#define GREG_MSI_MM_CAP4		(0x00000002 <<  0)
#define GREG_MSI_MM_CAP8		(0x00000003 <<  0)
#define GREG_MSI_MM_CAP16		(0x00000004 <<  0)
#define GREG_MSI_MM_CAP32		(0x00000005 <<  0)

#define GREG_PMCSR_DATA_1		0x008
#define GREG_PMCSR_DATA_2		0x00c
#define GREG_VI_CTRL			0x010
#define GREG_FGPI_CTRL			0x014

#define GREG_RSTU_CTRL			0x018
#define GREG_BOOT_READY			(0x00000001 << 13)
#define GREG_RESET_REQ			(0x00000001 << 12)
#define GREG_IP_RST_RELEASE		(0x00000001 << 11)
#define GREG_ADAPTER_RST_RELEASE	(0x00000001 << 10)
#define GREG_PCIE_CORE_RST_RELEASE	(0x00000001 <<  9)
#define GREG_BOOT_IP_RST_RELEASE	(0x00000001 <<  8)
#define GREG_BOOT_RST_RELEASE		(0x00000001 <<  7)
#define GREG_CGU_RST_RELEASE		(0x00000001 <<  6)
#define GREG_IP_RST_ASSERT		(0x00000001 <<  5)
#define GREG_ADAPTER_RST_ASSERT		(0x00000001 <<  4)
#define GREG_RST_ASSERT			(0x00000001 <<  3)
#define GREG_BOOT_IP_RST_ASSERT		(0x00000001 <<  2)
#define GREG_BOOT_RST_ASSERT		(0x00000001 <<  1)
#define GREG_CGU_RST_ASSERT		(0x00000001 <<  0)

#define GREG_I2C_CTRL			0x01c
#define GREG_I2C_SLAVE_ADDR		(0x0000007f <<  0)

#define GREG_OVFLW_CTRL			0x020
#define GREG_OVERFLOW_ENABLE		(0x00001fff <<  0)

#define GREG_TAG_ACK_FLEN		0x024
#define GREG_TAG_ACK_FLEN_1B		(0x00000000 <<  0)
#define GREG_TAG_ACK_FLEN_2B		(0x00000001 <<  0)
#define GREG_TAG_ACK_FLEN_4B		(0x00000002 <<  0)
#define GREG_TAG_ACK_FLEN_8B		(0x00000003 <<  0)

#define GREG_VIDEO_IN_CTRL		0x028

#define GREG_SPARE_1			0x02c
#define GREG_SPARE_2			0x030
#define GREG_SPARE_3			0x034
#define GREG_SPARE_4			0x038
#define GREG_SPARE_5			0x03c
#define GREG_SPARE_6			0x040
#define GREG_SPARE_7			0x044
#define GREG_SPARE_8			0x048
#define GREG_SPARE_9			0x04c
#define GREG_SPARE_10			0x050
#define GREG_SPARE_11			0x054
#define GREG_SPARE_12			0x058
#define GREG_SPARE_13			0x05c
#define GREG_SPARE_14			0x060
#define GREG_SPARE_15			0x064

#define GREG_FAIL_DISABLE		0x068
#define GREG_BOOT_FAIL_DISABLE		(0x00000001 <<  0)

#define GREG_SW_RST			0xff0
#define GREG_SW_RESET			(0x00000001 <<  0)




/* BAR = 20 bits */

/* -------------- PHI1 Registers -------------- */

#define PHI_1				0x00020000



#endif /* __SAA716x_REG_H */
