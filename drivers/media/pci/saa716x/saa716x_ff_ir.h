#ifndef __SAA716x_FF_IR_H
#define __SAA716x_FF_IR_H

extern int saa716x_ir_init(struct saa716x_dev *saa716x);
extern void saa716x_ir_exit(struct saa716x_dev *saa716x);
extern void saa716x_ir_handler(struct saa716x_dev *saa716x, u32 ir_cmd);

#endif /* __SAA716x_FF_IR_H */
