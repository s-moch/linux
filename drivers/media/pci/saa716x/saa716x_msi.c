#include <linux/delay.h>
#include "saa716x_reg.h"
#include "saa716x_priv.h"

#define MSI_CONFIG_REGS		49

int saa716x_msi_init(struct saa716x_dev *saa716x)
{
	u32 ena_l, ena_h, sta_l, sta_h;
	int i;
	u32 MSI_CONFIG_REG[49] = {
		MSI_CONFIG0,
		MSI_CONFIG1,
		MSI_CONFIG2,
		MSI_CONFIG3,
		MSI_CONFIG4,
		MSI_CONFIG5,
		MSI_CONFIG6,
		MSI_CONFIG7,
		MSI_CONFIG8,
		MSI_CONFIG9,
		MSI_CONFIG10,
		MSI_CONFIG11,
		MSI_CONFIG12,
		MSI_CONFIG13,
		MSI_CONFIG14,
		MSI_CONFIG15,
		MSI_CONFIG16,
		MSI_CONFIG17,
		MSI_CONFIG18,
		MSI_CONFIG19,
		MSI_CONFIG20,
		MSI_CONFIG21,
		MSI_CONFIG22,
		MSI_CONFIG23,
		MSI_CONFIG24,
		MSI_CONFIG25,
		MSI_CONFIG26,
		MSI_CONFIG27,
		MSI_CONFIG28,
		MSI_CONFIG29,
		MSI_CONFIG30,
		MSI_CONFIG31,
		MSI_CONFIG32,
		MSI_CONFIG33,
		MSI_CONFIG34,
		MSI_CONFIG35,
		MSI_CONFIG36,
		MSI_CONFIG37,
		MSI_CONFIG38,
		MSI_CONFIG39,
		MSI_CONFIG40,
		MSI_CONFIG41,
		MSI_CONFIG42,
		MSI_CONFIG43,
		MSI_CONFIG44,
		MSI_CONFIG45,
		MSI_CONFIG46,
		MSI_CONFIG47,
		MSI_CONFIG48
	};

	/* let HW take care of MSI race */
	SAA716x_WR(MSI, MSI_DELAY_TIMER, 0x0);

	/* INTA Polarity: Active High */
	SAA716x_WR(MSI, MSI_INTA_POLARITY, MSI_INTA_POLARITY_HIGH);

	/*
	 * IRQ Edge Rising: 25:24 = 0x01
	 * Traffic Class: 18:16 = 0x00
	 * MSI ID: 4:0 = 0x00
	 */
	for (i = 0; i < MSI_CONFIG_REGS; i++)
		SAA716x_WR(MSI, MSI_CONFIG_REG[i], 0x01000000);

	/* get Status */
	ena_l = SAA716x_RD(MSI, MSI_INT_ENA_L);
	ena_h = SAA716x_RD(MSI, MSI_INT_ENA_H);
	sta_l = SAA716x_RD(MSI, MSI_INT_STATUS_L);
	sta_h = SAA716x_RD(MSI, MSI_INT_STATUS_H);

	/* disable and clear enabled and asserted IRQ's */
	if (sta_l)
		SAA716x_WR(MSI, MSI_INT_STATUS_CLR_L, sta_l);

	if (sta_h)
		SAA716x_WR(MSI, MSI_INT_STATUS_CLR_H, sta_h);

	if (ena_l)
		SAA716x_WR(MSI, MSI_INT_ENA_CLR_L, ena_l);

	if (ena_h)
		SAA716x_WR(MSI, MSI_INT_ENA_CLR_H, ena_h);

	msleep(5);

	/* Check IRQ's really disabled */
	ena_l = SAA716x_RD(MSI, MSI_INT_ENA_L);
	ena_h = SAA716x_RD(MSI, MSI_INT_ENA_H);
	sta_l = SAA716x_RD(MSI, MSI_INT_STATUS_L);
	sta_h = SAA716x_RD(MSI, MSI_INT_STATUS_H);

	if ((ena_l == 0) && (ena_h == 0) && (sta_l == 0) && (sta_h == 0))
		return 0;
	else
		return -EIO;

	return 0;
}

void saa716x_msiint_disable(struct saa716x_dev *saa716x)
{
	SAA716x_WR(MSI, MSI_INT_ENA_L, 0x0);
	SAA716x_WR(MSI, MSI_INT_ENA_H, 0x0);
	SAA716x_WR(MSI, MSI_INT_STATUS_CLR_L, 0xffffffff);
	SAA716x_WR(MSI, MSI_INT_STATUS_CLR_L, 0x0000ffff);
}
EXPORT_SYMBOL_GPL(saa716x_msiint_disable);
