#ifndef __SAA716x_ADAP_H
#define __SAA716x_ADAP_H

extern int saa716x_dvb_init(struct saa716x_dev *saa716x);
extern void saa716x_dvb_exit(struct saa716x_dev *saa716x);
extern int saa716x_frontend_power(struct saa716x_dev *saa716x, u8 control);
extern int saa716x_frontend_reset(struct saa716x_dev *saa716x);

#endif /* __SAA716x_ADAP_H */
