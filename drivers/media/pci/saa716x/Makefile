saa716x_core-objs		 := saa716x_pci.o	\
				    saa716x_i2c.o	\
				    saa716x_cgu.o	\
				    saa716x_msi.o	\
				    saa716x_boot.o	\
				    saa716x_video.o	\
				    saa716x_fgpi.o

obj-$(CONFIG_DVB_SAA716X_CORE)    += saa716x_core.o
obj-$(CONFIG_DVB_SAA716X_BUDGET)  += saa716x_budget.o
obj-$(CONFIG_DVB_SAA716X_HYBRID)  += saa716x_hybrid.o

EXTRA_CFLAGS = -Idrivers/media/dvb/dvb-core/
