#include "saa716x_reg.h"
#include "saa716x_priv.h"
#include "saa716x_fgpi.h"
#include "saa716x_dma.h"

static const u32 mmu_pta_base[] = {
	MMU_PTA_BASE0,
	MMU_PTA_BASE1,
	MMU_PTA_BASE2,
	MMU_PTA_BASE3,
	MMU_PTA_BASE4,
	MMU_PTA_BASE5,
	MMU_PTA_BASE6,
	MMU_PTA_BASE7,
	MMU_PTA_BASE8,
	MMU_PTA_BASE9,
	MMU_PTA_BASE10,
	MMU_PTA_BASE11,
	MMU_PTA_BASE12,
	MMU_PTA_BASE13,
	MMU_PTA_BASE14,
	MMU_PTA_BASE15,
};

static const u32 mmu_dma_cfg[] = {
	MMU_DMA_CONFIG0,
	MMU_DMA_CONFIG1,
	MMU_DMA_CONFIG2,
	MMU_DMA_CONFIG3,
	MMU_DMA_CONFIG4,
	MMU_DMA_CONFIG5,
	MMU_DMA_CONFIG6,
	MMU_DMA_CONFIG7,
	MMU_DMA_CONFIG8,
	MMU_DMA_CONFIG9,
	MMU_DMA_CONFIG10,
	MMU_DMA_CONFIG11,
	MMU_DMA_CONFIG12,
	MMU_DMA_CONFIG13,
	MMU_DMA_CONFIG14,
	MMU_DMA_CONFIG15,
};

void saa716x_fgpiint_disable(struct saa716x_dev *saa716x)
{
	SAA716x_EPWR(FGPI0, 0xfe4, 0); /* disable FGPI IRQ */
	SAA716x_EPWR(FGPI1, 0xfe4, 0); /* disable FGPI IRQ */
	SAA716x_EPWR(FGPI2, 0xfe4, 0); /* disable FGPI IRQ */
	SAA716x_EPWR(FGPI3, 0xfe4, 0); /* disable FGPI IRQ */
	SAA716x_EPWR(FGPI0, 0xfe8, 0x7f); /* clear status */
	SAA716x_EPWR(FGPI1, 0xfe8, 0x7f); /* clear status */
	SAA716x_EPWR(FGPI2, 0xfe8, 0x7f); /* clear status */
	SAA716x_EPWR(FGPI3, 0xfe8, 0x7f); /* clear status */
}
EXPORT_SYMBOL_GPL(saa716x_fgpiint_disable);

static u32 saa716x_init_ptables(struct saa716x_dmabuf *dmabuf, int channel)
{
	struct saa716x_dev *saa716x = dmabuf->saa716x;

	u32 dma_ch, config, i;

	BUG_ON(dmabuf->mem_ptab_phys == NULL);

	for (i = 0; i < FGPI_BUFFERS; i++)
		BUG_ON(dmabuf[i].mem_ptab_phys);

	dma_ch = mmu_pta_base[channel]; /* DMA x */
	config = mmu_dma_cfg[channel]; /* DMACONFIGx */

	SAA716x_EPWR(MMU, config, (FGPI_BUFFERS - 1));
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA0_LSB(channel), PTA_LSB(dmabuf[0].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA0_MSB(channel), PTA_MSB(dmabuf[0].mem_ptab_phys)); /* High */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA1_LSB(channel), PTA_LSB(dmabuf[1].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA1_MSB(channel), PTA_MSB(dmabuf[1].mem_ptab_phys)); /* High */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA2_LSB(channel), PTA_LSB(dmabuf[2].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA2_MSB(channel), PTA_MSB(dmabuf[2].mem_ptab_phys)); /* High */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA3_LSB(channel), PTA_LSB(dmabuf[3].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA3_MSB(channel), PTA_MSB(dmabuf[3].mem_ptab_phys)); /* High */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA4_LSB(channel), PTA_LSB(dmabuf[4].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA4_MSB(channel), PTA_MSB(dmabuf[4].mem_ptab_phys)); /* High */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA5_LSB(channel), PTA_LSB(dmabuf[5].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA5_MSB(channel), PTA_MSB(dmabuf[5].mem_ptab_phys)); /* High */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA6_LSB(channel), PTA_LSB(dmabuf[6].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA6_MSB(channel), PTA_MSB(dmabuf[6].mem_ptab_phys)); /* High */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA7_LSB(channel), PTA_LSB(dmabuf[7].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, dma_ch + MMU_PTA7_MSB(channel), PTA_MSB(dmabuf[7].mem_ptab_phys)); /* High */

	return 0;
}
