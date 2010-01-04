#include <linux/delay.h>
#include "saa716x_reg.h"
#include "saa716x_priv.h"

#define CGU_CLKS	14
#define PLL_FREQ	2500

u32 CGU_FDC[CGU_CLKS] = {
	CGU_FDC_0,
	CGU_FDC_1,
	CGU_FDC_2,
	CGU_FDC_3,
	CGU_FDC_4,
	CGU_FDC_5,
	CGU_FDC_6,
	CGU_FDC_7,
	CGU_FDC_8,
	CGU_FDC_9,
	CGU_FDC_10,
	CGU_FDC_11,
	CGU_FDC_12,
	CGU_FDC_13,
};

u32 CGU_FREQ[CGU_CLKS];

int saa716x_cgu_init(struct saa716x_dev *saa716x)
{
	s16 M;
	s8 N;
	u8 i;
	u32 boot_div[14], freq[14];

	SAA716x_EPWR(CGU, CGU_PCR_0_6, CGU_PCR_RUN); /* GREG */
	SAA716x_EPWR(CGU, CGU_PCR_0_3, CGU_PCR_RUN); /* MMU */
	SAA716x_EPWR(CGU, CGU_PCR_0_4, CGU_PCR_RUN); /* DTL2MTL */
	SAA716x_EPWR(CGU, CGU_PCR_0_5, CGU_PCR_RUN); /* MSI */
	SAA716x_EPWR(CGU, CGU_PCR_3_2, CGU_PCR_RUN); /* I2C */
	SAA716x_EPWR(CGU, CGU_PCR_4_1, CGU_PCR_RUN); /* PHI */
	SAA716x_EPWR(CGU, CGU_PCR_0_7, CGU_PCR_RUN); /* GPIO */
	SAA716x_EPWR(CGU, CGU_PCR_2_1, CGU_PCR_RUN); /* SPI */
	SAA716x_EPWR(CGU, CGU_PCR_1_1, CGU_PCR_RUN); /* DCS */
	SAA716x_EPWR(CGU, CGU_PCR_3_1, CGU_PCR_RUN); /* BOOT */

	dprintk(SAA716x_DEBUG, 1, "Initial Clocks setup");

	for (i = 0; i < CGU_CLKS; i++) {
		boot_div[i] = SAA716x_EPRD(CGU, CGU_FDC[i]);

		N  =  (boot_div[i] >> 11) & 0xff;
		N *= -1;
		M  = ((boot_div[i] >> 3) & 0xff) + N;

		if (M)
			freq[i] = ((u32)N * PLL_FREQ) / (u32)M;
		else
			freq[i] = 0;

		dprintk(SAA716x_DEBUG, 1, "Clock %d, M=%d, Frequency=%d",
			i, M, freq[i]);
	}

	/* VI 0 */
	SAA716x_EPWR(CGU, CGU_PCR_5, CGU_PCR_RUN); /* Run */
	SAA716x_EPWR(CGU, CGU_SCR_5, CGU_SCR_ENF1); /* Switch */
	SAA716x_EPWR(CGU, CGU_FS1_5, 0x00000000); /* PLL Clk */
	SAA716x_EPWR(CGU, CGU_ESR_5, CGU_ESR_FD_EN); /* Frac div */

	/* VI 1 */
	SAA716x_EPWR(CGU, CGU_PCR_6, CGU_PCR_RUN);
	SAA716x_EPWR(CGU, CGU_SCR_6, CGU_SCR_ENF1);
	SAA716x_EPWR(CGU, CGU_FS1_6, 0x00000000);
	SAA716x_EPWR(CGU, CGU_ESR_6, CGU_ESR_FD_EN);

	/* FGPI 0 */
	SAA716x_EPWR(CGU, CGU_PCR_7, CGU_PCR_RUN);
	SAA716x_EPWR(CGU, CGU_SCR_7, CGU_SCR_ENF1);
	SAA716x_EPWR(CGU, CGU_FS1_7, 0x00000000);
	SAA716x_EPWR(CGU, CGU_ESR_7, CGU_ESR_FD_EN);

	/* FGPI 1 */
	SAA716x_EPWR(CGU, CGU_PCR_8, CGU_PCR_RUN);
	SAA716x_EPWR(CGU, CGU_SCR_8, CGU_SCR_ENF1);
	SAA716x_EPWR(CGU, CGU_FS1_8, 0x00000000);
	SAA716x_EPWR(CGU, CGU_ESR_8, CGU_ESR_FD_EN);

	/* FGPI 2 */
	SAA716x_EPWR(CGU, CGU_PCR_9, CGU_PCR_RUN);
	SAA716x_EPWR(CGU, CGU_SCR_9, CGU_SCR_ENF1);
	SAA716x_EPWR(CGU, CGU_FS1_9, 0x00000000);
	SAA716x_EPWR(CGU, CGU_ESR_9, CGU_ESR_FD_EN);

	/* FGPI 3 */
	SAA716x_EPWR(CGU, CGU_PCR_10, CGU_PCR_RUN);
	SAA716x_EPWR(CGU, CGU_SCR_10, CGU_SCR_ENF1);
	SAA716x_EPWR(CGU, CGU_FS1_10, 0x00000000);
	SAA716x_EPWR(CGU, CGU_ESR_10, CGU_ESR_FD_EN);

	/* AI 0 */
	SAA716x_EPWR(CGU, CGU_PCR_11, CGU_PCR_RUN);
	SAA716x_EPWR(CGU, CGU_SCR_11, CGU_SCR_ENF1);
	SAA716x_EPWR(CGU, CGU_FS1_11, 0x00000000);
	SAA716x_EPWR(CGU, CGU_ESR_11, CGU_ESR_FD_EN);

	/* AI 1 */
	SAA716x_EPWR(CGU, CGU_PCR_12, CGU_PCR_RUN);
	SAA716x_EPWR(CGU, CGU_SCR_12, CGU_SCR_ENF1);
	SAA716x_EPWR(CGU, CGU_FS1_12, 0x00000000);
	SAA716x_EPWR(CGU, CGU_ESR_12, CGU_ESR_FD_EN);

	msleep(1000); /* wait for PLL */

	return 0;
}
EXPORT_SYMBOL_GPL(saa716x_cgu_init);

#define	PORT_ALL	0
#define	PORT_VI0	1
#define	PORT_VI1	2
#define	PORT_FGPI0	3
#define	PORT_FGPI1	4
#define	PORT_FGPI2	5
#define	PORT_FGPI3	6
#define	PORT_AI0	7
#define	PORT_AI1	8

int saa716x_set_intclk(struct saa716x_dev *saa716x, u32 clk_domain)
{
	int delay = 1, err = 0;

	switch (clk_domain) {
	case PORT_VI0:
		SAA716x_EPWR(CGU, CGU_PCR_5, CGU_PCR_RUN); /* Run */
		SAA716x_EPWR(CGU, CGU_SCR_5, CGU_SCR_ENF1); /* Switch */
		SAA716x_EPWR(CGU, CGU_FS1_5, 0x00000000); /* PLL Clk */
		SAA716x_EPWR(CGU, CGU_ESR_5, CGU_ESR_FD_EN); /* Frac div */
		delay = 0;
		break;

	case PORT_VI1:
		SAA716x_EPWR(CGU, CGU_PCR_6, CGU_PCR_RUN); /* Run */
		SAA716x_EPWR(CGU, CGU_SCR_6, CGU_SCR_ENF1); /* Switch */
		SAA716x_EPWR(CGU, CGU_FS1_6, 0x00000000); /* PLL Clk */
		SAA716x_EPWR(CGU, CGU_ESR_6, CGU_ESR_FD_EN); /* Frac div */
		delay = 0;
		break;

	case PORT_FGPI0:
		SAA716x_EPWR(CGU, CGU_PCR_7, CGU_PCR_RUN); /* Run */
		SAA716x_EPWR(CGU, CGU_SCR_7, CGU_SCR_ENF1); /* Switch */
		SAA716x_EPWR(CGU, CGU_FS1_7, 0x00000000); /* PLL Clk */
		SAA716x_EPWR(CGU, CGU_ESR_7, CGU_ESR_FD_EN); /* Frac div */
		break;

	case PORT_FGPI1:
		SAA716x_EPWR(CGU, CGU_PCR_8, CGU_PCR_RUN); /* Run */
		SAA716x_EPWR(CGU, CGU_SCR_8, CGU_SCR_ENF1); /* Switch */
		SAA716x_EPWR(CGU, CGU_FS1_8, 0x00000000); /* PLL Clk */
		SAA716x_EPWR(CGU, CGU_ESR_8, CGU_ESR_FD_EN); /* Frac div */
		break;

	case PORT_FGPI2:
		SAA716x_EPWR(CGU, CGU_PCR_9, CGU_PCR_RUN); /* Run */
		SAA716x_EPWR(CGU, CGU_SCR_9, CGU_SCR_ENF1); /* Switch */
		SAA716x_EPWR(CGU, CGU_FS1_9, 0x00000000); /* PLL Clk */
		SAA716x_EPWR(CGU, CGU_ESR_9, CGU_ESR_FD_EN); /* Frac div */
		break;

	case PORT_FGPI3:
		SAA716x_EPWR(CGU, CGU_PCR_10, CGU_PCR_RUN); /* Run */
		SAA716x_EPWR(CGU, CGU_SCR_10, CGU_SCR_ENF1); /* Switch */
		SAA716x_EPWR(CGU, CGU_FS1_10, 0x00000000); /* PLL Clk */
		SAA716x_EPWR(CGU, CGU_ESR_10, CGU_ESR_FD_EN); /* Frac div */
		break;

	case PORT_AI0:
		SAA716x_EPWR(CGU, CGU_PCR_11, CGU_PCR_RUN); /* Run */
		SAA716x_EPWR(CGU, CGU_SCR_11, CGU_SCR_ENF1); /* Switch */
		SAA716x_EPWR(CGU, CGU_FS1_11, 0x00000000); /* PLL Clk */
		SAA716x_EPWR(CGU, CGU_ESR_11, CGU_ESR_FD_EN); /* Frac div */
		break;

	case PORT_AI1:
		SAA716x_EPWR(CGU, CGU_PCR_12, CGU_PCR_RUN); /* Run */
		SAA716x_EPWR(CGU, CGU_SCR_12, CGU_SCR_ENF1); /* Switch */
		SAA716x_EPWR(CGU, CGU_FS1_12, 0x00000000); /* PLL Clk */
		SAA716x_EPWR(CGU, CGU_ESR_12, CGU_ESR_FD_EN); /* Frac div */
		break;

	case PORT_ALL:
		/* VI 0 */
		SAA716x_EPWR(CGU, CGU_PCR_5, CGU_PCR_RUN); /* Run */
		SAA716x_EPWR(CGU, CGU_SCR_5, CGU_SCR_ENF1); /* Switch */
		SAA716x_EPWR(CGU, CGU_FS1_5, 0x00000000); /* PLL Clk */
		SAA716x_EPWR(CGU, CGU_ESR_5, CGU_ESR_FD_EN); /* Frac div */

		/* VI 1 */
		SAA716x_EPWR(CGU, CGU_PCR_6, CGU_PCR_RUN);
		SAA716x_EPWR(CGU, CGU_SCR_6, CGU_SCR_ENF1);
		SAA716x_EPWR(CGU, CGU_FS1_6, 0x00000000);
		SAA716x_EPWR(CGU, CGU_ESR_6, CGU_ESR_FD_EN);

		/* FGPI 0 */
		SAA716x_EPWR(CGU, CGU_PCR_7, CGU_PCR_RUN);
		SAA716x_EPWR(CGU, CGU_SCR_7, CGU_SCR_ENF1);
		SAA716x_EPWR(CGU, CGU_FS1_7, 0x00000000);
		SAA716x_EPWR(CGU, CGU_ESR_7, CGU_ESR_FD_EN);

		/* FGPI 1 */
		SAA716x_EPWR(CGU, CGU_PCR_8, CGU_PCR_RUN);
		SAA716x_EPWR(CGU, CGU_SCR_8, CGU_SCR_ENF1);
		SAA716x_EPWR(CGU, CGU_FS1_8, 0x00000000);
		SAA716x_EPWR(CGU, CGU_ESR_8, CGU_ESR_FD_EN);

		/* FGPI 2 */
		SAA716x_EPWR(CGU, CGU_PCR_9, CGU_PCR_RUN);
		SAA716x_EPWR(CGU, CGU_SCR_9, CGU_SCR_ENF1);
		SAA716x_EPWR(CGU, CGU_FS1_9, 0x00000000);
		SAA716x_EPWR(CGU, CGU_ESR_9, CGU_ESR_FD_EN);

		/* FGPI 3 */
		SAA716x_EPWR(CGU, CGU_PCR_10, CGU_PCR_RUN);
		SAA716x_EPWR(CGU, CGU_SCR_10, CGU_SCR_ENF1);
		SAA716x_EPWR(CGU, CGU_FS1_10, 0x00000000);
		SAA716x_EPWR(CGU, CGU_ESR_10, CGU_ESR_FD_EN);

		/* AI 0 */
		SAA716x_EPWR(CGU, CGU_PCR_11, CGU_PCR_RUN);
		SAA716x_EPWR(CGU, CGU_SCR_11, CGU_SCR_ENF1);
		SAA716x_EPWR(CGU, CGU_FS1_11, 0x00000000);
		SAA716x_EPWR(CGU, CGU_ESR_11, CGU_ESR_FD_EN);

		/* AI 1 */
		SAA716x_EPWR(CGU, CGU_PCR_12, CGU_PCR_RUN);
		SAA716x_EPWR(CGU, CGU_SCR_12, CGU_SCR_ENF1);
		SAA716x_EPWR(CGU, CGU_FS1_12, 0x00000000);
		SAA716x_EPWR(CGU, CGU_ESR_12, CGU_ESR_FD_EN);
		break;

	default:
		dprintk(SAA716x_ERROR, 1, "Unknown PORT: %d", clk_domain);
		delay = 0;
		err = -EINVAL;
		break;
	}

	/* Wait for the PLL to be ready */
	if (delay) {
		dprintk(SAA716x_DEBUG, 1, "Set PORT%d internal clock", clk_domain);
		msleep(1000);
	}

	return err;
}

int saa716x_set_extclk(struct saa716x_dev *saa716x, u32 port)
{
	int delay = 1, err = 0;

	switch (port) {
	case PORT_VI0:
		SAA716x_EPWR(CGU, CGU_FS1_5, 0x00000002);  /* select VIP0_CLK */
		SAA716x_EPWR(CGU, CGU_ESR_5, 0x00000000);  /* disable divider */
		delay = 0;
		break;

	case PORT_VI1:
		SAA716x_EPWR(CGU, CGU_FS1_6, 0x00000003);  /* select VIP1_CLK */
		SAA716x_EPWR(CGU, CGU_ESR_6, 0x00000000);  /* disable divider */
		delay = 0;
		break;

	case PORT_FGPI0:
		SAA716x_EPWR(CGU, CGU_FS1_7, 0x00000004);  /* select FGPI0_CLK */
		SAA716x_EPWR(CGU, CGU_ESR_7, 0x00000000);  /* disable divider */
		break;

	case PORT_FGPI1:
		SAA716x_EPWR(CGU, CGU_FS1_8, 0x00000005);  /* select FGPI1_CLK */
		SAA716x_EPWR(CGU, CGU_ESR_8, 0x00000000);  /* disable divider */
		break;

	case PORT_FGPI2:
		SAA716x_EPWR(CGU, CGU_FS1_9, 0x00000006);  /* select FGPI2_CLK */
		SAA716x_EPWR(CGU, CGU_ESR_9, 0x00000000);  /* disable divider */
		break;

	case PORT_FGPI3:
		SAA716x_EPWR(CGU, CGU_FS1_10, 0x00000007);  /* select FGPI3_CLK */
		SAA716x_EPWR(CGU, CGU_ESR_10, 0x00000000);  /* disable divider */
		break;

	case PORT_AI0:
		SAA716x_EPWR(CGU, CGU_FS1_11, 0x00000008);  /* select AI0_CLK */
		SAA716x_EPWR(CGU, CGU_ESR_11, 0x00000000);  /* disable divider */
		break;

	case PORT_AI1:
		SAA716x_EPWR(CGU, CGU_FS1_12, 0x00000009);  /* select AI1_CLK */
		SAA716x_EPWR(CGU, CGU_ESR_12, 0x00000000);  /* disable divider */
		break;

	default:
		dprintk(SAA716x_DEBUG, 1, "Error: unsupported port: %d", port);
		delay = 0;
		err = -EINVAL;
		break;
	}

	/* Wait for the PLL to be ready */
	if (delay) {
		dprintk(SAA716x_DEBUG, 1, "Set PORT%d internal clock", port);
		msleep(1000);
	}

	return err;
}

enum saa716x_clk {
	SAA716x_CLK_PSS		= 0,
	SAA716x_CLK_DCS,
	SAA716x_CLK_SPI,
	SAA716x_CLK_I2C,
	SAA716x_CLK_PHI,
	SAA716x_CLK_VI0,
	SAA716x_CLK_VI1,
	SAA716x_CLK_FGPI0,
	SAA716x_CLK_FGPI1,
	SAA716x_CLK_FGPI2,
	SAA716x_CLK_FGPI3,
	SAA716x_CLK_AI0,
	SAA716x_CLK_AI1,
	SAA716x_CLK_PHY
};

int saa716x_set_clk(struct saa716x_dev *saa716x, enum saa716x_clk clk_domain, u32 frequency)
{
	u32 M = 1, N = 1;
	u32 reset = 0, freq, cur_div;
	s8 ucN, ucM, sub, add, lsb;
	int i;

	/* calculate a new divider */
	do {
		M = (N * PLL_FREQ) / frequency;
		if (M == 0)
			N++;
	} while (M == 0);

	/* calculate real frequency */
	freq = (N * PLL_FREQ) / M;

	ucN = 0xFF & N;
	ucM = 0xFF & M;
	sub = -ucN;
	add = ucM - ucN;
	lsb = 4; /* run */

	/* enable stretching for N/M smaller than 1/2 */
	if (((10 * N) / M) < 5)
        	lsb |= 1; /* Add stretching */

	cur_div   = sub & 0xff;
	cur_div <<= 8;
	cur_div  |= add & 0xff;
	cur_div <<= 3;
	cur_div  |= lsb;

	dprintk(SAA716x_DEBUG, 1, "INFO: SetClockFrequency %d f %d set f %d, N = %d, M = %d, divider 0x%x",
		clk_domain,
		frequency,
		freq,
		N,
		M,
		cur_div);

	/* add reset to the frequency divider, update register and wait */
	SAA716x_EPWR(CGU, CGU_FDC[clk_domain], cur_div | 0x2);

	/* monitor reset disable */
	for (i = 0; i < CGU_CLKS; i++) {
		msleep(10);
		reset = SAA716x_EPRD(CGU, CGU_FREQ[clk_domain]);
		if (cur_div == reset)
			break;
	}

	/* if not changed, disable reset */
	if (cur_div != reset)
		SAA716x_EPWR(CGU, CGU_FREQ[clk_domain], cur_div);

	return 0;
}

u32 saa716x_get_clk(struct saa716x_dev *saa716x, enum saa716x_clk clk_domain)
{
	return CGU_FREQ[clk_domain];
}

