#include "saa716x_reg.h"
#include "saa716x_priv.h"

#define CGU_CLKS	15
#define PLL_FREQ	2500

int saa716x_cgu_init(struct saa716x_dev *saa716x)
{
	int M, N;
	u8 i;
	u32 boot_div[15], freq[15];
	u32 CGU_FDC[15] = {
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

	SAA716x_WR(CGU, CGU_PCR_0_6, CGU_PCR_RUN); /* GREG */
	SAA716x_WR(CGU, CGU_PCR_0_3, CGU_PCR_RUN); /* MMU */
	SAA716x_WR(CGU, CGU_PCR_0_4, CGU_PCR_RUN); /* DTL2MTL */
	SAA716x_WR(CGU, CGU_PCR_0_5, CGU_PCR_RUN); /* MSI */
	SAA716x_WR(CGU, CGU_PCR_3_2, CGU_PCR_RUN); /* I2C */
	SAA716x_WR(CGU, CGU_PCR_4_1, CGU_PCR_RUN); /* PHI */
	SAA716x_WR(CGU, CGU_PCR_0_7, CGU_PCR_RUN); /* GPIO */
	SAA716x_WR(CGU, CGU_PCR_2_1, CGU_PCR_RUN); /* SPI */
	SAA716x_WR(CGU, CGU_PCR_1_1, CGU_PCR_RUN); /* DCS */
	SAA716x_WR(CGU, CGU_PCR_3_1, CGU_PCR_RUN); /* BOOT */

	dprintk(SAA716x_DEBUG, 1, "Initial Clocks setup");

	for (i = 0; i < CGU_CLKS; i++) {
		boot_div[i] = SAA716x_RD(CGU, CGU_FDC[i]);

		N  =  (boot_div[i] >> 11) & 0xff;
		N *= -1;
		M  = ((boot_div[i] >> 3) & 0xff) + N;

		if (M)
			freq[i] = (N * PLL_FREQ) / M;
		else
			freq[i] = 0;

		dprintk(SAA716x_DEBUG, 1, "Clock %d, M=%d, Frequency=%d",
			i, M, freq[i]);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(saa716x_cgu_init);
