// SPDX-License-Identifier: GPL-2.0+

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/scatterlist.h>
#include <linux/vmalloc.h>
#include <asm/page.h>
#include <asm/pgtable.h>

#include "saa716x_dma.h"
#include "saa716x_priv.h"

/*  Allocates one page of memory, which is stores the data of one
 *  716x page table. The result gets stored in the passed DMA buffer
 *  structure.
 */
static int saa716x_allocate_ptable(struct saa716x_dmabuf *dmabuf)
{
	struct saa716x_dev *saa716x	= dmabuf->saa716x;
	struct pci_dev *pdev		= saa716x->pdev;

	pci_dbg(saa716x->pdev, "SG Page table allocate");
	dmabuf->mem_ptab_virt = (void *) __get_free_page(GFP_KERNEL);

	if (dmabuf->mem_ptab_virt == NULL) {
		pci_err(saa716x->pdev, "ERROR: Out of pages !");
		return -ENOMEM;
	}

	dmabuf->mem_ptab_phys = dma_map_single(&pdev->dev,
						dmabuf->mem_ptab_virt,
						SAA716x_PAGE_SIZE,
						DMA_TO_DEVICE);

	if (dmabuf->mem_ptab_phys == 0) {
		pci_err(saa716x->pdev, "ERROR: map memory failed !");
		return -ENOMEM;
	}

	return 0;
}

static void saa716x_free_ptable(struct saa716x_dmabuf *dmabuf)
{
	struct saa716x_dev *saa716x	= dmabuf->saa716x;
	struct pci_dev *pdev		= saa716x->pdev;

	BUG_ON(dmabuf == NULL);
	pci_dbg(saa716x->pdev, "SG Page table free");

	/* free physical PCI memory */
	if (dmabuf->mem_ptab_phys != 0) {
		dma_unmap_single(&pdev->dev,
				 dmabuf->mem_ptab_phys,
				 SAA716x_PAGE_SIZE,
				 DMA_TO_DEVICE);

		dmabuf->mem_ptab_phys = 0;
	}

	/* free kernel memory */
	if (dmabuf->mem_ptab_virt != NULL) {
		free_page((unsigned long) dmabuf->mem_ptab_virt);
		dmabuf->mem_ptab_virt = NULL;
	}
}

static void saa716x_dmabuf_sgfree(struct saa716x_dmabuf *dmabuf)
{
	struct saa716x_dev *saa716x = dmabuf->saa716x;

	BUG_ON(dmabuf == NULL);
	pci_dbg(saa716x->pdev, "SG free");

	dmabuf->mem_virt = NULL;
	if (dmabuf->mem_virt_noalign != NULL) {
		if (dmabuf->dma_type == SAA716x_DMABUF_INT)
			vfree(dmabuf->mem_virt_noalign);

		dmabuf->mem_virt_noalign = NULL;
	}

	if (dmabuf->sg_list != NULL) {
		kfree(dmabuf->sg_list);
		dmabuf->sg_list = NULL;
	}
}

/* Create a SG, the needed memory gets allocated */
static int saa716x_dmabuf_sgalloc(struct saa716x_dmabuf *dmabuf, int size)
{
	struct saa716x_dev *saa716x	= dmabuf->saa716x;
	struct scatterlist *list;
	struct page *pg;

	int i, pages;

	BUG_ON(!(size > 0));
	BUG_ON(dmabuf == NULL);
	pci_dbg(saa716x->pdev, "SG allocate");

	if ((size % SAA716x_PAGE_SIZE) != 0) /* calculate required pages */
		pages = size / SAA716x_PAGE_SIZE + 1;
	else
		pages = size / SAA716x_PAGE_SIZE;

	/* Allocate memory for SG list */
	dmabuf->sg_list = kcalloc(pages, sizeof(struct scatterlist),
				  GFP_KERNEL);
	if (dmabuf->sg_list == NULL)
		return -ENOMEM;

	pci_dbg(saa716x->pdev, "Initializing SG table");
	sg_init_table(dmabuf->sg_list, pages);

	/* allocate memory, unaligned */
	dmabuf->mem_virt_noalign = vzalloc((pages + 1) * SAA716x_PAGE_SIZE);
	if (dmabuf->mem_virt_noalign == NULL)
		return -ENOMEM;

	/* align memory to page */
	dmabuf->mem_virt =
		(void *) PAGE_ALIGN(((unsigned long) dmabuf->mem_virt_noalign));

	dmabuf->list_len = pages; /* scatterlist length */
	list = dmabuf->sg_list;

	pci_dbg(saa716x->pdev, "Allocating SG pages");
	for (i = 0; i < pages; i++) {
		pg = vmalloc_to_page(dmabuf->mem_virt + i * SAA716x_PAGE_SIZE);
		BUG_ON(pg == NULL);
		sg_set_page(&list[i], pg, SAA716x_PAGE_SIZE, 0);
	}

	pci_dbg(saa716x->pdev, "Done!");
	return 0;
}

/*  Fill the "page table" page with the pointers to the specified SG buffer */
static void saa716x_dmabuf_sgpagefill(struct saa716x_dmabuf *dmabuf,
			 struct scatterlist *sg_list, int pages, int offset)
{
	struct saa716x_dev *saa716x	= dmabuf->saa716x;
	struct pci_dev *pdev		= saa716x->pdev;
	struct scatterlist *sg_cur;

	u32 *page;
	int i, j, k = 0;
	dma_addr_t addr = 0;

	BUG_ON(dmabuf == NULL);
	BUG_ON(sg_list == NULL);
	BUG_ON(pages == 0);
	pci_dbg(saa716x->pdev, "SG page fill");

	/* make page writable for the PC */
	dma_sync_single_for_cpu(&pdev->dev, dmabuf->mem_ptab_phys,
				SAA716x_PAGE_SIZE, DMA_TO_DEVICE);
	page = dmabuf->mem_ptab_virt;

	/* create page table */
	for (i = 0; i < pages; i++) {
		sg_cur = &sg_list[i];

		if (i == 0)
			dmabuf->offset =
			  (sg_cur->length + sg_cur->offset) % SAA716x_PAGE_SIZE;
		else
			BUG_ON(sg_cur->offset != 0);

		for (j = 0; (j * SAA716x_PAGE_SIZE) < sg_dma_len(sg_cur); j++) {

			if ((offset + sg_cur->offset) >= SAA716x_PAGE_SIZE) {
				offset -= SAA716x_PAGE_SIZE;
				continue;
			}

			addr = ((u64)sg_dma_address(sg_cur)) +
			       (j * SAA716x_PAGE_SIZE) - sg_cur->offset;

			BUG_ON(addr == 0);
			page[k * 2] = (u32)addr; /* Low */
			page[k * 2 + 1] = (u32)(((u64) addr) >> 32); /* High */
			BUG_ON(page[k * 2] % SAA716x_PAGE_SIZE);
			k++;
		}
	}

	for (; k < (SAA716x_PAGE_SIZE / 8); k++) {
		page[k * 2] = (u32) addr;
		page[k * 2 + 1] = (u32) (((u64) addr) >> 32);
	}

	/* make "page table" page writable for the PC */
	dma_sync_single_for_device(&pdev->dev,
				   dmabuf->mem_ptab_phys,
				   SAA716x_PAGE_SIZE,
				   DMA_TO_DEVICE);

}

void saa716x_dmabufsync_dev(struct saa716x_dmabuf *dmabuf)
{
	struct saa716x_dev *saa716x	= dmabuf->saa716x;
	struct pci_dev *pdev		= saa716x->pdev;

	pci_dbg(saa716x->pdev, "DMABUF sync DEVICE");
	BUG_ON(dmabuf->sg_list == NULL);

	dma_sync_sg_for_device(&pdev->dev,
			       dmabuf->sg_list,
			       dmabuf->list_len,
			       DMA_FROM_DEVICE);

}

void saa716x_dmabufsync_cpu(struct saa716x_dmabuf *dmabuf)
{
	struct saa716x_dev *saa716x	= dmabuf->saa716x;
	struct pci_dev *pdev		= saa716x->pdev;

	pci_dbg(saa716x->pdev, "DMABUF sync CPU");
	BUG_ON(dmabuf->sg_list == NULL);

	dma_sync_sg_for_cpu(&pdev->dev,
			    dmabuf->sg_list,
			    dmabuf->list_len,
			    DMA_FROM_DEVICE);
}

/* Allocates a DMA buffer for the specified external linear buffer. */
int saa716x_dmabuf_alloc(struct saa716x_dev *saa716x,
			 struct saa716x_dmabuf *dmabuf, int size)
{
	struct pci_dev *pdev		= saa716x->pdev;

	int ret;

	BUG_ON(saa716x == NULL);
	BUG_ON(dmabuf == NULL);
	BUG_ON(!(size > 0));

	dmabuf->dma_type		= SAA716x_DMABUF_INT;

	dmabuf->mem_virt_noalign	= NULL;
	dmabuf->mem_virt		= NULL;
	dmabuf->mem_ptab_phys		= 0;
	dmabuf->mem_ptab_virt		= NULL;

	dmabuf->list_len		= 0;
	dmabuf->saa716x			= saa716x;

	/* Allocate page table */
	ret = saa716x_allocate_ptable(dmabuf);
	if (ret < 0) {
		pci_err(saa716x->pdev, "PT alloc failed, Out of memory");
		goto err1;
	}

	/* Allocate buffer as SG */
	ret = saa716x_dmabuf_sgalloc(dmabuf, size);
	if (ret < 0) {
		pci_err(saa716x->pdev, "SG alloc failed");
		goto err2;
	}

	ret = dma_map_sg(&pdev->dev, dmabuf->sg_list, dmabuf->list_len,
			 DMA_FROM_DEVICE);
	if (ret < 0) {
		pci_err(saa716x->pdev, "SG map failed");
		goto err3;
	}

	saa716x_dmabuf_sgpagefill(dmabuf, dmabuf->sg_list, ret, 0);

	return 0;
err3:
	saa716x_dmabuf_sgfree(dmabuf);
err2:
	saa716x_free_ptable(dmabuf);
err1:
	return ret;
}

void saa716x_dmabuf_free(struct saa716x_dev *saa716x,
			 struct saa716x_dmabuf *dmabuf)
{
	struct pci_dev *pdev		= saa716x->pdev;

	BUG_ON(saa716x == NULL);
	BUG_ON(dmabuf == NULL);

	dma_unmap_sg(&pdev->dev, dmabuf->sg_list, dmabuf->list_len,
		     DMA_FROM_DEVICE);
	saa716x_dmabuf_sgfree(dmabuf);
	saa716x_free_ptable(dmabuf);
}
