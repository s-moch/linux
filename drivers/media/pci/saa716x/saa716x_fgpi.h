#ifndef __SAA716x_FGPI_H
#define __SAA716x_FGPI_H

#define FGPI_BUFFERS		8
#define PTA_LSB(__mem)		((u32 ) *(__mem))
#define PTA_MSB(__mem)		((u32 ) (*(__mem) >> 32))

#define BAM_DMA_BUF_MODE_BASE		0x0d8
#define BAM_DMA_BUF_MODE_OFFSET		0x24

#define BAM_DMA_BUF_MODE(__ch)		(BAM_DMA_BUF_MODE_BASE + (BAM_DMA_BUF_MODE_OFFSET * __ch))

#define BAM_FGPI_ADDR_OFFST_BASE	0x0dc
#define BAM_FGPI_ADDR_OFFST_OFFSET	0x24

#define BAM_FGPI_ADDR_OFFSET(__ch)	(BAM_FGPI_ADDR_OFFST_BASE + (BAM_FGPI_ADDR_OFFST_OFFSET * __ch))

#define BAM_FGPI_ADDR_OFFST_0(__ch)	BAM_FGPI_ADDR_OFFSET(__ch) + 0x00
#define BAM_FGPI_ADDR_OFFST_1(__ch)	BAM_FGPI_ADDR_OFFSET(__ch) + 0x04
#define BAM_FGPI_ADDR_OFFST_2(__ch)	BAM_FGPI_ADDR_OFFSET(__ch) + 0x08
#define BAM_FGPI_ADDR_OFFST_3(__ch)	BAM_FGPI_ADDR_OFFSET(__ch) + 0x0c
#define BAM_FGPI_ADDR_OFFST_4(__ch)	BAM_FGPI_ADDR_OFFSET(__ch) + 0x10
#define BAM_FGPI_ADDR_OFFST_5(__ch)	BAM_FGPI_ADDR_OFFSET(__ch) + 0x14
#define BAM_FGPI_ADDR_OFFST_6(__ch)	BAM_FGPI_ADDR_OFFSET(__ch) + 0x18
#define BAM_FGPI_ADDR_OFFST_7(__ch)	BAM_FGPI_ADDR_OFFSET(__ch) + 0x1c

struct saa716x_dmabuf;

/*
 * Port supported streams
 *
 * FGPI_AUDIO_STREAM
 * FGPI_VIDEO_STREAM
 * FGPI_VBI_STREAM
 * FGPI_TRANSPORT_STREAM
 * FGPI_PROGRAM_STREAM
 */
enum fgpi_stream_type {
	FGPI_AUDIO_STREAM	= 0x01,
	FGPI_VIDEO_STREAM	= 0x02,
	FGPI_VBI_STREAM		= 0x04,
	FGPI_TRANSPORT_STREAM	= 0x08,
	FGPI_PROGRAM_STREAM	= 0x10
};

/*
 * Stream port flags
 *
 * FGPI_ODD_FIELD
 * FGPI_EVEN_FIELD
 * FGPI_HD_0
 * FGPI_HD_1
 * FGPI_PAL
 * FGPI_NTSC
 */
enum fgpi_stream_flags {
	FGPI_ODD_FIELD		= 0x0001,
	FGPI_EVEN_FIELD		= 0x0002,
	FGPI_INTERLACED		= 0x0004,
	FGPI_HD0		= 0x0010,
	FGPI_HD1		= 0x0020,
	FGPI_PAL		= 0x0040,
	FGPI_NTSC		= 0x0080,
	FGPI_NO_SCALER		= 0x0100,
};

/*
 * Stream port parameters
 * bits: Bits per sample
 * samples: samples perline
 * lines: number of lines
 * pitch: stream pitch in bytes
 * offset: offset to first valid line
 */
struct fgpi_stream_params {
	u32			bits;
	u32			samples;
	u32			lines;

	s32			pitch;

	u32			offset;
	u32			page_tables;

	enum fgpi_stream_flags	stream_flags;
	enum fgpi_stream_type	stream_type;
};

struct saa716x_dmabuf;

extern void saa716x_fgpiint_disable(struct saa716x_dmabuf *dmabuf, int channel);

#endif /* __SAA716x_FGPI_H */
