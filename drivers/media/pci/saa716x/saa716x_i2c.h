#ifndef __SAA716x_I2C_H
#define __SAA716x_I2C_H

struct saa716x_dev;

enum saa716x_i2c_rate {
	SAA716x_I2C_RATE_400 = 1,
	SAA716x_I2C_RATE_100,
};

struct saa716x_i2c {
	struct i2c_adapter		i2c_adapter;
	struct mutex			i2c_lock;
	struct saa716x_dev		*saa716x;
	u32				i2c_dev;

	enum saa716x_i2c_rate		i2c_rate; /* run time */
	wait_queue_head_t		i2c_wq;
	u32				int_stat;
};

extern int saa716x_i2c_init(struct saa716x_dev *saa716x);
extern int saa716x_i2c_exit(struct saa716x_dev *saa716x);
extern void saa716x_i2cint_disable(struct saa716x_dev *saa716x);

#endif /* __SAA716x_I2C_H */
