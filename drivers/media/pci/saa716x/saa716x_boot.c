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

/* Internal Bootscript configuration */
static void saa716x_int_boot(struct saa716x_dev *saa716x)
{
	/* #1 Configure PCI COnfig space
	 * GREG_JETSTR_CONFIG_0
	 */
	SAA716x_WR(GREG, GREG_SUBSYS_CONFIG, saa716x->pdev->subsystem_vendor);

	/* GREG_JETSTR_CONFIG_1
	 * pmcsr_scale:7 = 0x00
	 * pmcsr_scale:6 = 0x00
	 * pmcsr_scale:5 = 0x00
	 * pmcsr_scale:4 = 0x00
	 * pmcsr_scale:3 = 0x00
	 * pmcsr_scale:2 = 0x00
	 * pmcsr_scale:1 = 0x00
	 * pmcsr_scale:0 = 0x00
	 * BAR mask = 20 bit
	 * BAR prefetch = no
	 * MSI capable = 32 messages
	 */
	SAA716x_WR(GREG, GREG_MSI_BAR_PMCSR, 0x00001005);

	/* GREG_JETSTR_CONFIG_2
	 * pmcsr_data:3 = 0x0
	 * pmcsr_data:2 = 0x0
	 * pmcsr_data:1 = 0x0
	 * pmcsr_data:0 = 0x0
	 */
	SAA716x_WR(GREG, GREG_PMCSR_DATA_1, 0x00000000);

	/* GREG_JETSTR_CONFIG_3
	 * pmcsr_data:7 = 0x0
	 * pmcsr_data:6 = 0x0
	 * pmcsr_data:5 = 0x0
	 * pmcsr_data:4 = 0x0
	 */
	SAA716x_WR(GREG, GREG_PMCSR_DATA_2, 0x00000000);

	/* #2 Release GREG resets
	 * ip_rst_an
	 * dpa1_rst_an
	 * jetsream_reset_an
	 */
	SAA716x_WR(GREG, GREG_RSTU_CTRL, 0x00000e00);

	/* #3 GPIO Setup
	 * GPIO 25:24 = Output
	 * GPIO Output "0" after Reset
	 */
	SAA716x_WR(GPIO, GPIO_OEN, 0xfcffffff);

	/* #4 Custom stuff goes in here */

	/* #5 Disable CGU Clocks
	 * except for PHY, Jetstream, DPA1, DCS, Boot, GREG
	 * CGU_PCR_0_3: pss_mmu_clk:0 = 0x0
	 */
	SAA716x_WR(CGU, CGU_PCR_0_3, 0x00000006);

	/* CGU_PCR_0_4: pss_dtl2mtl_mmu_clk:0 = 0x0 */
	SAA716x_WR(CGU, CGU_PCR_0_4, 0x00000006);

	/* CGU_PCR_0_5: pss_msi_ck:0 = 0x0 */
	SAA716x_WR(CGU, CGU_PCR_0_5, 0x00000006);

	/* CGU_PCR_0_7: pss_gpio_clk:0 = 0x0 */
	SAA716x_WR(CGU, CGU_PCR_0_7, 0x00000006);

	/* CGU_PCR_2_1: spi_clk:0 = 0x0 */
	SAA716x_WR(CGU, CGU_PCR_2_1, 0x00000006);

	/* CGU_PCR_3_2: i2c_clk:0 = 0x0 */
	SAA716x_WR(CGU, CGU_PCR_3_2, 0x00000006);

	/* CGU_PCR_4_1: phi_clk:0 = 0x0 */
	SAA716x_WR(CGU, CGU_PCR_4_1, 0x00000006);

	/* CGU_PCR_5: vip0_clk:0 = 0x0 */
	SAA716x_WR(CGU, CGU_PCR_5, 0x00000006);

	/* CGU_PCR_6: vip1_clk:0 = 0x0 */
	SAA716x_WR(CGU, CGU_PCR_6, 0x00000006);

	/* CGU_PCR_7: fgpi0_clk:0 = 0x0 */
	SAA716x_WR(CGU, CGU_PCR_7, 0x00000006);

	/* CGU_PCR_8: fgpi1_clk:0 = 0x0 */
	SAA716x_WR(CGU, CGU_PCR_8, 0x00000006);

	/* CGU_PCR_9: fgpi2_clk:0 = 0x0 */
	SAA716x_WR(CGU, CGU_PCR_9, 0x00000006);

	/* CGU_PCR_10: fgpi3_clk:0 = 0x0 */
	SAA716x_WR(CGU, CGU_PCR_10, 0x00000006);

	/* CGU_PCR_11: ai0_clk:0 = 0x0 */
	SAA716x_WR(CGU, CGU_PCR_11, 0x00000006);

	/* CGU_PCR_12: ai1_clk:0 = 0x0 */
	SAA716x_WR(CGU, CGU_PCR_12, 0x00000006);

	/* #6 Set GREG boot_ready = 0x1 */
	SAA716x_WR(GREG, GREG_RSTU_CTRL, 0x00002000);

	/* #7 Disable GREG CGU Clock */
	SAA716x_WR(CGU, CGU_PCR_0_6, 0x00000006);

	/* End of Bootscript command ?? */
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
