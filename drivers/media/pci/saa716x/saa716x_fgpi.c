#include <linux/kernel.h>

#include "saa716x_mod.h"

#include "saa716x_fgpi_reg.h"
#include "saa716x_dma_reg.h"
#include "saa716x_msi_reg.h"

#include "saa716x_dma.h"
#include "saa716x_fgpi.h"
#include "saa716x_spi.h"
#include "saa716x_priv.h"

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

static const u32 fgpi_ch[] = {
	FGPI0,
	FGPI1,
	FGPI2,
	FGPI3
};

static const u32 bamdma_bufmode[] = {
	BAM_FGPI0_DMA_BUF_MODE,
	BAM_FGPI1_DMA_BUF_MODE,
	BAM_FGPI2_DMA_BUF_MODE,
	BAM_FGPI3_DMA_BUF_MODE
};

static const u32 msi_int_tagack[] = {
	MSI_INT_TAGACK_FGPI_0,
	MSI_INT_TAGACK_FGPI_1,
	MSI_INT_TAGACK_FGPI_2,
	MSI_INT_TAGACK_FGPI_3
};

static const u32 msi_int_ovrflw[] = {
	MSI_INT_OVRFLW_FGPI_0,
	MSI_INT_OVRFLW_FGPI_1,
	MSI_INT_OVRFLW_FGPI_2,
	MSI_INT_OVRFLW_FGPI_3
};

static const u32 msi_int_avint[] = {
	MSI_INT_AVINT_FGPI_0,
	MSI_INT_AVINT_FGPI_1,
	MSI_INT_AVINT_FGPI_2,
	MSI_INT_AVINT_FGPI_3
};

void saa716x_fgpiint_disable(struct saa716x_dmabuf *dmabuf, int channel)
{
	struct saa716x_dev *saa716x = dmabuf->saa716x;

	u32 fgpi_port;

	fgpi_port = fgpi_ch[channel];

	SAA716x_EPWR(fgpi_port, INT_ENABLE, 0); /* disable FGPI IRQ */
	SAA716x_EPWR(fgpi_port, INT_CLR_STATUS, 0x7f); /* clear status */
}
EXPORT_SYMBOL_GPL(saa716x_fgpiint_disable);

int saa716x_fgpi_get_write_index(struct saa716x_dev *saa716x, u32 fgpi_index)
{
	u32 fgpi_base;
	u32 buf_mode_reg;
	u32 buf_mode;

 	switch (fgpi_index) {
	case 0: /* FGPI_0 */
		fgpi_base = FGPI0;
		buf_mode_reg = BAM_FGPI0_DMA_BUF_MODE;
		break;

	case 1: /* FGPI_1 */
		fgpi_base = FGPI1;
		buf_mode_reg = BAM_FGPI1_DMA_BUF_MODE;
		break;

	case 2: /* FGPI_2 */
		fgpi_base = FGPI2;
		buf_mode_reg = BAM_FGPI2_DMA_BUF_MODE;
		break;

	case 3: /* FGPI_3 */
		fgpi_base = FGPI3;
		buf_mode_reg = BAM_FGPI3_DMA_BUF_MODE;
		break;

	default:
		printk(KERN_ERR "%s: unexpected fgpi %u\n",
		       __func__, fgpi_index);
		return -1;
	}

	buf_mode = SAA716x_EPRD(BAM, buf_mode_reg);
	if (saa716x->revision < 2) {
		/* workaround for revision 1: restore buffer numbers on BAM */
		SAA716x_EPWR(fgpi_base, INT_CLR_STATUS, 0x7F);
		SAA716x_EPWR(BAM, buf_mode_reg, buf_mode | 7);
	}
	return (buf_mode >> 3) & 0x7;
}
EXPORT_SYMBOL_GPL(saa716x_fgpi_get_write_index);

static u32 saa716x_init_ptables(struct saa716x_dmabuf *dmabuf, int channel)
{
	struct saa716x_dev *saa716x = dmabuf->saa716x;

	u32 config, i;

	for (i = 0; i < FGPI_BUFFERS; i++)
		BUG_ON((dmabuf[i].mem_ptab_phys == 0));

	config = mmu_dma_cfg[channel]; /* DMACONFIGx */

	SAA716x_EPWR(MMU, config, (FGPI_BUFFERS - 1));
	SAA716x_EPWR(MMU, MMU_PTA0_LSB(channel), PTA_LSB(dmabuf[0].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, MMU_PTA0_MSB(channel), PTA_MSB(dmabuf[0].mem_ptab_phys)); /* High */
	SAA716x_EPWR(MMU, MMU_PTA1_LSB(channel), PTA_LSB(dmabuf[1].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, MMU_PTA1_MSB(channel), PTA_MSB(dmabuf[1].mem_ptab_phys)); /* High */
	SAA716x_EPWR(MMU, MMU_PTA2_LSB(channel), PTA_LSB(dmabuf[2].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, MMU_PTA2_MSB(channel), PTA_MSB(dmabuf[2].mem_ptab_phys)); /* High */
	SAA716x_EPWR(MMU, MMU_PTA3_LSB(channel), PTA_LSB(dmabuf[3].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, MMU_PTA3_MSB(channel), PTA_MSB(dmabuf[3].mem_ptab_phys)); /* High */
	SAA716x_EPWR(MMU, MMU_PTA4_LSB(channel), PTA_LSB(dmabuf[4].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, MMU_PTA4_MSB(channel), PTA_MSB(dmabuf[4].mem_ptab_phys)); /* High */
	SAA716x_EPWR(MMU, MMU_PTA5_LSB(channel), PTA_LSB(dmabuf[5].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, MMU_PTA5_MSB(channel), PTA_MSB(dmabuf[5].mem_ptab_phys)); /* High */
	SAA716x_EPWR(MMU, MMU_PTA6_LSB(channel), PTA_LSB(dmabuf[6].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, MMU_PTA6_MSB(channel), PTA_MSB(dmabuf[6].mem_ptab_phys)); /* High */
	SAA716x_EPWR(MMU, MMU_PTA7_LSB(channel), PTA_LSB(dmabuf[7].mem_ptab_phys)); /* Low */
	SAA716x_EPWR(MMU, MMU_PTA7_MSB(channel), PTA_MSB(dmabuf[7].mem_ptab_phys)); /* High */

	return 0;
}

int saa716x_fgpi_setparams(struct saa716x_dmabuf *dmabuf,
			   struct fgpi_stream_params *stream_params,
			   int port)
{
	struct saa716x_dev *saa716x = dmabuf->saa716x;

	u32 fgpi_port, buf_mode, val, mid;
	u32 D1_XY_END, offst_1, offst_2;
	int i = 0;

	fgpi_port = fgpi_ch[port];
	buf_mode = bamdma_bufmode[port];

	/* Reset FGPI block */
	SAA716x_EPWR(fgpi_port, FGPI_SOFT_RESET, FGPI_SOFTWARE_RESET);

	/* Reset DMA channel */
	SAA716x_EPWR(BAM, buf_mode, 0x00000040);
	saa716x_init_ptables(dmabuf, saa716x->fgpi[port].dma_channel);


	/* monitor BAM reset */
	val = SAA716x_EPRD(BAM, buf_mode);
	while (val && (i < 100)) {
		msleep(30);
		val = SAA716x_EPRD(BAM, buf_mode);
		i++;
	}

	if (val) {
		dprintk(SAA716x_ERROR, 1, "Error: BAM FGPI Reset failed!");
		return -EIO;
	}

	/* set buffer count */
	SAA716x_EPWR(BAM, buf_mode, FGPI_BUFFERS - 1);

	/* initialize all available address offsets */
	SAA716x_EPWR(BAM, BAM_FGPI_ADDR_OFFST_0(port), 0x0);
	SAA716x_EPWR(BAM, BAM_FGPI_ADDR_OFFST_1(port), 0x0);
	SAA716x_EPWR(BAM, BAM_FGPI_ADDR_OFFST_2(port), 0x0);
	SAA716x_EPWR(BAM, BAM_FGPI_ADDR_OFFST_3(port), 0x0);
	SAA716x_EPWR(BAM, BAM_FGPI_ADDR_OFFST_4(port), 0x0);
	SAA716x_EPWR(BAM, BAM_FGPI_ADDR_OFFST_5(port), 0x0);
	SAA716x_EPWR(BAM, BAM_FGPI_ADDR_OFFST_6(port), 0x0);
	SAA716x_EPWR(BAM, BAM_FGPI_ADDR_OFFST_7(port), 0x0);

	/* get module ID */
	mid = SAA716x_EPRD(fgpi_port, FGPI_MODULE_ID);
	if (mid != 0x14b0100)
		dprintk(SAA716x_ERROR, 1, "FGPI Id<%04x> is not supported", mid);

	/* Initialize FGPI block */
	SAA716x_EPWR(fgpi_port, FGPI_REC_SIZE, stream_params->samples * (stream_params->bits / 8));
	SAA716x_EPWR(fgpi_port, FGPI_STRIDE, stream_params->pitch);

	offst_1 = 0;
	offst_2 = 0;
	switch (stream_params->stream_type) {
	case FGPI_TRANSPORT_STREAM:
		SAA716x_EPWR(fgpi_port, FGPI_CONTROL, 0x00000080);
		SAA716x_EPWR(fgpi_port, FGPI_SIZE, stream_params->lines);
		break;

	case FGPI_PROGRAM_STREAM:
		SAA716x_EPWR(fgpi_port, FGPI_CONTROL, 0x00000088);
		SAA716x_EPWR(fgpi_port, FGPI_SIZE, stream_params->lines);
		break;

	case FGPI_VIDEO_STREAM:
		SAA716x_EPWR(fgpi_port, FGPI_CONTROL, 0x00000088);
		SAA716x_EPWR(fgpi_port, FGPI_D1_XY_START, 0x00000002);

		if ((stream_params->stream_flags & FGPI_INTERLACED) &&
		    (stream_params->stream_flags & FGPI_ODD_FIELD) &&
		    (stream_params->stream_flags & FGPI_EVEN_FIELD)) {

			SAA716x_EPWR(fgpi_port, FGPI_SIZE, stream_params->lines / 2);
			SAA716x_EPWR(fgpi_port, FGPI_STRIDE, 768 * 4); /* interlaced stride of 2 lines */

			D1_XY_END  = (stream_params->samples << 16);
			D1_XY_END |= (stream_params->lines / 2) + 2;

			if (stream_params->stream_flags & FGPI_PAL)
				offst_1 = 768 * 2;
			else
				offst_2 = 768 * 2;

		} else {
			SAA716x_EPWR(fgpi_port, FGPI_SIZE, stream_params->lines);
			SAA716x_EPWR(fgpi_port, FGPI_STRIDE, 768 * 2); /* stride of 1 line */

			D1_XY_END  = stream_params->samples << 16;
			D1_XY_END |= stream_params->lines + 2;
		}

		SAA716x_EPWR(fgpi_port, FGPI_D1_XY_END, D1_XY_END);
		break;

	default:
		SAA716x_EPWR(fgpi_port, FGPI_CONTROL, 0x00000080);
		break;
	}

	SAA716x_EPWR(fgpi_port, FGPI_BASE_1, ((saa716x->fgpi[port].dma_channel) << 21) + offst_1);
	SAA716x_EPWR(fgpi_port, FGPI_BASE_2, ((saa716x->fgpi[port].dma_channel) << 21) + offst_2);

	return 0;
}

int saa716x_fgpi_start(struct saa716x_dev *saa716x, int port,
		       struct fgpi_stream_params *stream_params)
{
	u32 fgpi_port;
	u32 config;
	u32 val;
	u32 i;

	fgpi_port = fgpi_ch[port];

	SAA716x_EPWR(fgpi_port, FGPI_INTERFACE, 0);
	msleep(10);

	if (saa716x_fgpi_setparams(saa716x->fgpi[port].dma_buf, stream_params, port) != 0) {
		return -EIO;
	}

	config = mmu_dma_cfg[saa716x->fgpi[port].dma_channel]; /* DMACONFIGx */

	val = SAA716x_EPRD(MMU, config);
	SAA716x_EPWR(MMU, config, val & ~0x40);
	SAA716x_EPWR(MMU, config, val | 0x40);

	SAA716x_EPWR(fgpi_port, INT_ENABLE, 0x7F);

	val = SAA716x_EPRD(MMU, config);
	i = 0;
	while (i < 500) {
		if (val & 0x80)
			break;
		msleep(10);
		val = SAA716x_EPRD(MMU, config);
		i++;
	}

	if (!(val & 0x80)) {
		dprintk(SAA716x_ERROR, 1, "Error: PTE pre-fetch failed!");
		return -EIO;
	}

	val = SAA716x_EPRD(fgpi_port, FGPI_CONTROL);
	val |= 0x3000;

	saa716x_set_clk_external(saa716x, saa716x->fgpi[port].dma_channel);

	SAA716x_EPWR(fgpi_port, FGPI_CONTROL, val);

	SAA716x_EPWR(MSI, MSI_INT_ENA_SET_L, msi_int_tagack[port]);

	return 0;
}

int saa716x_fgpi_stop(struct saa716x_dev *saa716x, int port)
{
	u32 fgpi_port;
	u32 val;

	fgpi_port = fgpi_ch[port];

	SAA716x_EPWR(MSI, MSI_INT_ENA_CLR_L, msi_int_tagack[port]);

	val = SAA716x_EPRD(fgpi_port, FGPI_CONTROL);
	val &= ~0x3000;
	SAA716x_EPWR(fgpi_port, FGPI_CONTROL, val);

	saa716x_set_clk_internal(saa716x, saa716x->fgpi[port].dma_channel);

	return 0;
}

int saa716x_fgpi_init(struct saa716x_dev *saa716x, int port,
		      void (*worker)(unsigned long))
{
	int i;
	int ret;

	saa716x->fgpi[port].dma_channel = port + 6;
	for (i = 0; i < FGPI_BUFFERS; i++)
	{
		/* TODO: what is a good size for TS DMA buffer? */
		ret = saa716x_dmabuf_alloc(saa716x, &saa716x->fgpi[port].dma_buf[i], 16 * SAA716x_PAGE_SIZE);
		if (ret < 0) {
			return ret;
		}
	}
	saa716x->fgpi[port].saa716x = saa716x;
	tasklet_init(&saa716x->fgpi[port].tasklet, worker,
		     (unsigned long)&saa716x->fgpi[port]);
	saa716x->fgpi[port].read_index = 0;

	return 0;
}

int saa716x_fgpi_exit(struct saa716x_dev *saa716x, int port)
{
	int i;

	tasklet_kill(&saa716x->fgpi[port].tasklet);
	for (i = 0; i < FGPI_BUFFERS; i++)
	{
		saa716x_dmabuf_free(saa716x, &saa716x->fgpi[port].dma_buf[i]);
	}

	return 0;
}
