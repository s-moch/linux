#ifndef __SAA716x_DMA_H
#define __SAA716x_DMA_H

#define SAA716x_PAGE_SIZE	4096

enum saa716x_dma_type {
	SAA716x_DMABUF_EXT_LIN, /* Linear external */
	SAA716x_DMABUF_EXT_SG, /* SG external */
	SAA716x_DMABUF_INT /* Linear internal */
};

struct saa716x_dev;

struct saa716x_dmabuf {
	enum saa716x_dma_type	dma_type;

	void			*mem_virt_noalign;
	void			*mem_virt; /* page aligned */
	dma_addr_t		mem_ptab_phys;
	void			*mem_ptab_virt;
	void			*sg_list; /* SG list */

	struct saa716x_dev	*saa716x;

	int			list_len; /* buffer len */
	int			offset; /* page offset */
};

extern int saa716x_dmabuf_alloc(struct saa716x_dev *saa716x,
				struct saa716x_dmabuf *dmabuf,
				int size);
extern void saa716x_dmabuf_free(struct saa716x_dev *saa716x,
				struct saa716x_dmabuf *dmabuf);

extern void saa716x_dmabufsync_dev(struct saa716x_dmabuf *dmabuf);
extern void saa716x_dmabufsync_cpu(struct saa716x_dmabuf *dmabuf);

#endif /* __SAA716x_DMA_H */
