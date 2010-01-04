#ifndef __SAA716x_VIP_REG_H
#define __SAA716x_VIP_REG_H

/* -------------- VIP Registers -------------- */

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

#endif /* __SAA716x_VIP_REG_H */
