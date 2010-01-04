#include <linux/delay.h>
#include "saa716x_reg.h"
#include "saa716x_priv.h"

static int saa716x_ext_boot(struct saa716x_dev *saa716x)
{
	/* Write to GREG Subsystem ID */
	SAA716x_WR(GREG, GREG_SUBSYS_CONFIG, saa716x->pdev->subsystem_vendor);

	/* GREG_JETSTR_CONFIG_1 */
	SAA716x_WR(GREG, GREG_MSI_BAR_PMCSR, GREG_MSI_MM_CAP32 | GREG_BAR_WIDTH_20 | GREG_MSI_MM_CAP32);

	/* GREG_JETSTR_CONFIG_2 */
	SAA716x_WR(GREG, GREG_PMCSR_DATA_1, 0);

	/* GREG_JETSTR_CONFIG_3 */
	SAA716x_WR(GREG, GREG_PMCSR_DATA_2, 0);

	/* Release GREG Resets */
	SAA716x_WR(GREG, GREG_RSTU_CTRL, GREG_IP_RST_RELEASE | GREG_ADAPTER_RST_RELEASE | GREG_PCIE_CORE_RST_RELEASE);

	/* Disable Logical A/V channels */
	SAA716x_WR(GREG, GPIO_OEN, 0xfcffffff);

	/* Disable all clocks except PHY, Adapter, DCSN and Boot */
	SAA716x_WR(CGU, CGU_PCR_0_3, 0x00000006);
	SAA716x_WR(CGU, CGU_PCR_0_4, 0x00000006);
	SAA716x_WR(CGU, CGU_PCR_0_7, 0x00000006);
	SAA716x_WR(CGU, CGU_PCR_2_1, 0x00000006);
	SAA716x_WR(CGU, CGU_PCR_3_2, 0x00000006);
	SAA716x_WR(CGU, CGU_PCR_4_1, 0x00000006);
	SAA716x_WR(CGU, CGU_PCR_5,   0x00000006);
	SAA716x_WR(CGU, CGU_PCR_6,   0x00000006);
	SAA716x_WR(CGU, CGU_PCR_7,   0x00000006);
	SAA716x_WR(CGU, CGU_PCR_8,   0x00000006);
	SAA716x_WR(CGU, CGU_PCR_9,   0x00000006);
	SAA716x_WR(CGU, CGU_PCR_10,  0x00000006);
	SAA716x_WR(CGU, CGU_PCR_11,  0x00000006);
	SAA716x_WR(CGU, CGU_PCR_12,  0x00000006);

	/* Set GREG Boot Ready */
	SAA716x_WR(GREG, GREG_RSTU_CTRL, GREG_BOOT_READY);

	/* Disable GREG Clock */
	SAA716x_WR(CGU, CGU_PCR_0_6, 0x00000006);

	return 0;
}

static int saa716x_int_boot(struct saa716x_dev *saa716x)
{
	return 0;
}

int saa716x_core_boot(struct saa716x_dev *saa716x)
{
	struct saa716x_config *config = saa716x->config;

	switch (config->boot_mode) {
	case SAA716x_EXT_BOOT:
		dprintk(SAA716x_DEBUG, 1, "Using External Boot from config");
		saa716x_ext_boot(saa716x);
		break;
	case SAA716x_INT_BOOT:
		dprintk(SAA716x_DEBUG, 1, "Using Internal Boot from config");
		saa716x_int_boot(saa716x);
		break;
	case SAA716x_CGU_BOOT:
	default:
		dprintk(SAA716x_ERROR, 1, "Using CGU Setup");
		saa716x_cgu_init(saa716x);
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(saa716x_core_boot);

int saa716x_jetpack_init(struct saa716x_dev *saa716x)
{
	/* Reset all blocks */
	SAA716x_WR(MSI, MSI_SW_RST, MSI_SW_RESET);
	SAA716x_WR(MMU, MMU_SW_RST, MMU_SW_RESET);
	SAA716x_WR(BAM, BAM_SW_RST, BAM_SW_RESET);

	switch (saa716x->pdev->device) {
	case SAA7162:
		dprintk(SAA716x_DEBUG, 1, "SAA%02x Decoder disable", saa716x->pdev->device);
		SAA716x_WR(GPIO, GPIO_OEN, 0xfcffffff);
		SAA716x_WR(GPIO, GPIO_WR,  0x00000000); /* Disable decoders */
		msleep(10);
		SAA716x_WR(GPIO, GPIO_WR,  0x03000000); /* Enable decoders */
		break;
	case SAA7161:
		dprintk(SAA716x_DEBUG, 1, "SAA%02x Decoder disable", saa716x->pdev->device);
		SAA716x_WR(GPIO, GPIO_OEN, 0xfeffffff);
		SAA716x_WR(GPIO, GPIO_WR,  0x00000000); /* Disable decoders */
		msleep(10);
		SAA716x_WR(GPIO, GPIO_WR,  0x01000000); /* Enable decoder */
		break;
	case SAA7160:
		saa716x->i2c_rate = SAA716x_I2C_RATE_100;
		break;
	default:
		dprintk(SAA716x_ERROR, 1, "Unknown device (0x%02x)", saa716x->pdev->device);
		return -ENODEV;
	}

	/* General setup for MMU */
	SAA716x_WR(MMU, MMU_MODE, 0x14);
	dprintk(SAA716x_DEBUG, 1, "SAA%02x Jetpack Successfully initialized", saa716x->pdev->device);

	return 0;
}
EXPORT_SYMBOL(saa716x_jetpack_init);
