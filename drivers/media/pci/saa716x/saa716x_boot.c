#include <linux/delay.h>

#include "saa716x_mod.h"

#include "saa716x_greg_reg.h"
#include "saa716x_vip_reg.h"
#include "saa716x_aip_reg.h"
#include "saa716x_msi_reg.h"
#include "saa716x_dma_reg.h"
#include "saa716x_gpio_reg.h"
#include "saa716x_fgpi_reg.h"
#include "saa716x_dcs_reg.h"

#include "saa716x_boot.h"
#include "saa716x_priv.h"

static void saa716x_core_reset(struct saa716x_dev *saa716x)
{
	dprintk(SAA716x_DEBUG, 1, "RESET Modules");

	/* GREG */
	SAA716x_EPWR(GREG, GREG_SW_RST, GREG_SW_RESET);

	/* VIP */
	SAA716x_EPWR(VI0, VI_MODE, SOFT_RESET);
	SAA716x_EPWR(VI1, VI_MODE, SOFT_RESET);

	/* FGPI */
	SAA716x_EPWR(FGPI0, FGPI_SOFT_RESET, FGPI_SOFTWARE_RESET);
	SAA716x_EPWR(FGPI1, FGPI_SOFT_RESET, FGPI_SOFTWARE_RESET);
	SAA716x_EPWR(FGPI2, FGPI_SOFT_RESET, FGPI_SOFTWARE_RESET);
	SAA716x_EPWR(FGPI3, FGPI_SOFT_RESET, FGPI_SOFTWARE_RESET);

	/* AIP */
	SAA716x_EPWR(AI0, AI_CTL, AI_RESET);
	SAA716x_EPWR(AI1, AI_CTL, AI_RESET);

	/* GPIO */
	SAA716x_EPWR(GPIO, GPIO_SW_RST, GPIO_SW_RESET);

	/* BAM */
	SAA716x_EPWR(BAM, BAM_SW_RST, BAM_SW_RESET);

	/* MMU */
	SAA716x_EPWR(MMU, MMU_SW_RST, MMU_SW_RESET);

	/* MSI */
	SAA716x_EPWR(MSI, MSI_SW_RST, MSI_SW_RESET);
}

static void saa716x_bus_report(struct pci_dev *pdev, int enable)
{
	u32 reg;

	pci_read_config_dword(pdev, 0x04, &reg);
	if (enable)
		reg |= 0x00000100; /* enable SERR */
	else
		reg &= 0xfffffeff; /* disable SERR */
	pci_write_config_dword(pdev, 0x04, reg);

	pci_read_config_dword(pdev, 0x58, &reg);
	reg &= 0xfffffffd;
	pci_write_config_dword(pdev, 0x58, reg);
}

int saa716x_jetpack_init(struct saa716x_dev *saa716x)
{
	saa716x_core_reset(saa716x);
	SAA716x_EPWR(GREG, GREG_RSTU_CTRL, GREG_BOOT_READY);

	/*
	 * configure PHY through config space not to report
	 * non-fatal error messages to avoid problems with
	 * quirky BIOS'es
	 */
	saa716x_bus_report(saa716x->pdev, 0);

	/*
	 * create time out for blocks that have no clock
	 * helps with lower bitrates on FGPI
	 */
	SAA716x_EPWR(DCS, DCSC_CTRL, ENABLE_TIMEOUT);

	switch (saa716x->pdev->device) {
	case SAA7162:
		dprintk(SAA716x_DEBUG, 1, "SAA%02x Decoder disable", saa716x->pdev->device);
		SAA716x_EPWR(GPIO, GPIO_OEN, 0xfcffffff);
		SAA716x_EPWR(GPIO, GPIO_WR,  0x00000000); /* Disable decoders */
		msleep(10);
		SAA716x_EPWR(GPIO, GPIO_WR,  0x03000000); /* Enable decoders */
		break;
	case SAA7161:
		dprintk(SAA716x_DEBUG, 1, "SAA%02x Decoder disable", saa716x->pdev->device);
		SAA716x_EPWR(GPIO, GPIO_OEN, 0xfeffffff);
		SAA716x_EPWR(GPIO, GPIO_WR,  0x00000000); /* Disable decoders */
		msleep(10);
		SAA716x_EPWR(GPIO, GPIO_WR,  0x01000000); /* Enable decoder */
		break;
	case SAA7160:
		break;
	default:
		dprintk(SAA716x_ERROR, 1, "Unknown device (0x%02x)", saa716x->pdev->device);
		return -ENODEV;
	}

	/* General setup for MMU */
	SAA716x_EPWR(MMU, MMU_MODE, 0x14);
	dprintk(SAA716x_DEBUG, 1, "SAA%02x Jetpack Successfully initialized", saa716x->pdev->device);

	return 0;
}
EXPORT_SYMBOL(saa716x_jetpack_init);
