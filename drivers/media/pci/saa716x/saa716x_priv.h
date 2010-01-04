#ifndef __SAA716x_PRIV_H
#define __SAA716x_PRIV_H

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/mutex.h>

#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/i2c.h>
#include "saa716x_i2c.h"

#define SAA716x_ERROR		0
#define SAA716x_NOTICE		1
#define SAA716x_INFO		2
#define SAA716x_DEBUG		3

#define SAA716x_DEV		(saa716x)->num
#define SAA716x_VERBOSE		(saa716x)->verbose

#define dprintk(__x, __y, __fmt, __arg...) do {								\
	if (__y) {											\
		if	((SAA716x_VERBOSE > SAA716x_ERROR) && (SAA716x_VERBOSE > __x))			\
			printk(KERN_ERR "%s (%d): " __fmt "\n" , __func__ , SAA716x_DEV , ##__arg);	\
		else if	((SAA716x_VERBOSE > SAA716x_NOTICE) && (SAA716x_VERBOSE > __x))			\
			printk(KERN_NOTICE "%s (%d): " __fmt "\n" , __func__ , SAA716x_DEV , ##__arg);	\
		else if ((SAA716x_VERBOSE > SAA716x_INFO) && (SAA716x_VERBOSE > __x))			\
			printk(KERN_INFO "%s (%d): " __fmt "\n" , __func__ , SAA716x_DEV , ##__arg);	\
		else if ((SAA716x_VERBOSE > SAA716x_DEBUG) && (SAA716x_VERBOSE > __x))			\
			printk(KERN_DEBUG "%s (%d): " __fmt "\n" , __func__ , SAA716x_DEV , ##__arg);	\
	} else {											\
		if (SAA716x_VERBOSE > __x)								\
			printk(__fmt , ##__arg);							\
	}												\
} while(0)


#define NXP_SEMICONDUCTOR	0x1131
#define SAA7160			0x7160
#define SAA7161			0x7161
#define SAA7162			0x7162

#define MAKE_ENTRY(__subven, __subdev, __chip, __configptr) {		\
		.vendor		= NXP_SEMICONDUCTOR,			\
		.device		= (__chip),				\
		.subvendor	= (__subven),				\
		.subdevice	= (__subdev),				\
		.driver_data	= (unsigned long) (__configptr)		\
}

#define SAA716x_WR(__offst, __addr, __data)	writel((__data), (saa716x->mmio + (__offst + __addr)))
#define SAA716x_RD(__offst, __addr)		readl((saa716x->mmio + (__offst + __addr)))

struct saa716x_dev;

typedef int (*saa716x_load_config_t)(struct saa716x_dev *saa716x);

enum saa716x_boot_mode {
	SAA716x_EXT_BOOT = 1,
	SAA716x_INT_BOOT,
	SAA716x_CGU_BOOT,
};

struct saa716x_config {
	char				*model_name;
	char				*dev_type;

	enum saa716x_boot_mode		boot_mode;

	saa716x_load_config_t		load_config;
};

struct saa716x_dev {
	struct saa716x_config		*config;
	struct pci_dev			*pdev;

	int				num; /* device count */
	int				verbose;

	u8 				revision;

	/* PCI */
	unsigned long			addr;
	void __iomem			*mmio;
	u32				len;

	/* I2C */
	struct saa716x_i2c		i2c[2];
	u32				i2c_rate; /* init time */

	/* DMA */
};

/* PCI */
extern int saa716x_pci_init(struct saa716x_dev *saa716x);
extern void saa716x_pci_exit(struct saa716x_dev *saa716x);

/* MSI */
extern int saa716x_msi_init(struct saa716x_dev *saa716x);
extern void saa716x_msi_exit(struct saa716x_dev *saa716x);
extern void saa716x_msiint_disable(struct saa716x_dev *saa716x);

/* SPI */
extern int saa716x_spi_init(struct saa716x_dev *saa716x);
extern void saa716x_spi_exit(struct saa716x_dev *saa716x);

/* DMA */
extern int saa716x_dma_init(struct saa716x_dev *saa716x);
extern void saa716x_dma_exit(struct saa716x_dev *saa716x);

/* DVB */
extern int saa716x_dvb_init(struct saa716x_dev *saa716x);
extern void saa716x_dvb_exit(struct saa716x_dev *saa716x);

/* AUDIO */
extern int saa716x_audio_init(struct saa716x_dev *saa716x);
extern void saa716x_audio_exit(struct saa716x_dev *saa716x);

/* Boot */
extern int saa716x_core_boot(struct saa716x_dev *saa716x);
extern int saa716x_jetpack_init(struct saa716x_dev *saa716x);

/* CGU */
extern int saa716x_cgu_init(struct saa716x_dev *saa716x);

/* VIP */
extern void saa716x_vipint_disable(struct saa716x_dev *saa716x);

/* FGPI */
extern void saa716x_fgpiint_disable(struct saa716x_dev *saa716x);

#endif /* __SAA716x_PRIV_H */
