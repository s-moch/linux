#ifndef __SAA716x_FF_IR_H
#define __SAA716x_FF_IR_H

/* infrared remote control */
struct infrared {
	u16			key_map[128];
	struct input_dev	*input_dev;
	char			input_phys[32];
	struct timer_list	keyup_timer;
	struct tasklet_struct	tasklet;
	u32			command;
	u32			device_mask;
	u8			protocol;
	u16			last_key;
	u16			last_toggle;
	bool			key_pressed;
};

struct saa716x_ff_dev;

extern int saa716x_ir_init(struct saa716x_ff_dev *saa716x_ff);
extern void saa716x_ir_exit(struct saa716x_ff_dev *saa716x_ff);
extern void saa716x_ir_handler(struct saa716x_ff_dev *saa716x_ff, u32 ir_cmd);

#endif /* __SAA716x_FF_IR_H */
