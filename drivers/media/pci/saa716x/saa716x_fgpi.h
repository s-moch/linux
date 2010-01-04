#ifndef __SAA716x_FGPI_H
#define __SAA716x_FGPI_H

#define FGPI_BUFFERS		8
#define PTA_LSB(__mem)		((u32 ) *(__mem))
#define PTA_MSB(__mem)		((u32 ) (*(__mem) >> 32))

#endif /* __SAA716x_FGPI_H */
