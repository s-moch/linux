#include <linux/bitops.h>

#include "dmxdev.h"
#include "dvbdev.h"
#include "dvb_demux.h"
#include "dvb_frontend.h"

#include "saa716x_priv.h"
#include "saa716x_adap.h"
#include "saa716x_i2c.h"
#include "saa716x_gpio.h"

void saa716x_dma_start(struct saa716x_dev *saa716x)
{
	dprintk(SAA716x_DEBUG, 1, "SAA716x Start DMA engine");
}

void saa716x_dma_stop(struct saa716x_dev *saa716x)
{
	dprintk(SAA716x_DEBUG, 1, "SAA716x Stop DMA engine");
}

static int saa716x_dvb_start_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx		= dvbdmxfeed->demux;
	struct saa716x_adapter *saa716x_adap	= dvbdmx->priv;
	struct saa716x_dev *saa716x		= saa716x_adap->saa716x;

	dprintk(SAA716x_DEBUG, 1, "SAA716x DVB Start feed");
	if (!dvbdmx->dmx.frontend) {
		dprintk(SAA716x_DEBUG, 1, "no frontend ?");
		return -EINVAL;
	}
	saa716x_adap->feeds++;
	dprintk(SAA716x_DEBUG, 1, "SAA716x start feed, feeds=%d",
		saa716x_adap->feeds);

	if (saa716x_adap->feeds == 1) {
		dprintk(SAA716x_DEBUG, 1, "SAA716x start feed & dma");
		printk("saa716x start feed & dma\n");
		saa716x_dma_start(saa716x);
	}

	return saa716x_adap->feeds;
}

static int saa716x_dvb_stop_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx		= dvbdmxfeed->demux;
	struct saa716x_adapter *saa716x_adap	= dvbdmx->priv;
	struct saa716x_dev *saa716x		= saa716x_adap->saa716x;

	dprintk(SAA716x_DEBUG, 1, "SAA716x DVB Stop feed");
	if (!dvbdmx->dmx.frontend) {
		dprintk(SAA716x_DEBUG, 1, "no frontend ?");
		return -EINVAL;
	}
	saa716x_adap->feeds--;
	if (saa716x_adap->feeds == 0) {
		dprintk(SAA716x_DEBUG, 1, "saa716x stop feed and dma");
		printk("saa716x stop feed and dma\n");
		saa716x_dma_stop(saa716x);
	}

	return 0;
}

static int saa716x_frontend_power(struct saa716x_dev *saa716x, u8 DEV, u8 control)
{
	struct saa716x_adapter *adapter		= &saa716x->saa716x_adap[DEV];
	struct saa716x_config *config		= saa716x->config;
	struct saa716x_adap_config *adap_cfg	= &config->adap_config[DEV];

	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) Power ON", DEV);
	saa716x_gpio_ctl(saa716x, adap_cfg->power_ctl);
	saa716x_gpio_bits(saa716x, adap_cfg->power_ctl);

	return 0;
}

static int saa716x_frontend_reset(struct saa716x_dev *saa716x, u8 DEV)
{
	struct saa716x_adapter *adapter		= &saa716x->saa716x_adap[DEV];
	struct saa716x_config *config		= saa716x->config;
	struct saa716x_adap_config *adap_cfg	= &config->adap_config[DEV];

	dprintk(SAA716x_DEBUG, 1, "Adapter (%d) RESET", DEV);
	saa716x_gpio_ctl(saa716x, adap_cfg->reset_ctl);
	saa716x_gpio_bits(saa716x, adap_cfg->reset_ctl);

	return 0;
}

int __devinit saa716x_dvb_init(struct saa716x_dev *saa716x)
{
	struct saa716x_adapter *saa716x_adap = saa716x->saa716x_adap;
	struct saa716x_config *config = saa716x->config;
	int result, i;

	for (i = 0; i < config->adapters; i++) {

		dprintk(SAA716x_DEBUG, 1, "dvb_register_adapter");
		if (dvb_register_adapter(&saa716x_adap->dvb_adapter,
					 "SAA716x dvb adapter", THIS_MODULE,
					 &saa716x->pdev->dev) < 0) {

			dprintk(SAA716x_ERROR, 1, "Error registering adapter");
			return -ENODEV;
		}
		saa716x_adap->dvb_adapter.priv		= saa716x_adap;
		saa716x_adap->demux.dmx.capabilities	= DMX_TS_FILTERING	|
							  DMX_SECTION_FILTERING	|
							  DMX_MEMORY_BASED_FILTERING;

		saa716x_adap->demux.priv		= saa716x_adap;
		saa716x_adap->demux.filternum		= 256;
		saa716x_adap->demux.feednum		= 256;
		saa716x_adap->demux.start_feed		= saa716x_dvb_start_feed;
		saa716x_adap->demux.stop_feed		= saa716x_dvb_stop_feed;
		saa716x_adap->demux.write_to_decoder	= NULL;

		dprintk(SAA716x_DEBUG, 1, "dvb_dmx_init");
		if ((result = dvb_dmx_init(&saa716x_adap->demux)) < 0) {
			dprintk(SAA716x_ERROR, 1, "dvb_dmx_init failed, ERROR=%d", result);
			goto err0;
		}

		saa716x_adap->dmxdev.filternum		= 256;
		saa716x_adap->dmxdev.demux		= &saa716x_adap->demux.dmx;
		saa716x_adap->dmxdev.capabilities	= 0;

		dprintk(SAA716x_DEBUG, 1, "dvb_dmxdev_init");
		if ((result = dvb_dmxdev_init(&saa716x_adap->dmxdev,
					      &saa716x_adap->dvb_adapter)) < 0) {

			dprintk(SAA716x_ERROR, 1, "dvb_dmxdev_init failed, ERROR=%d", result);
			goto err1;
		}

		saa716x_adap->fe_hw.source = DMX_FRONTEND_0;

		if ((result = saa716x_adap->demux.dmx.add_frontend(&saa716x_adap->demux.dmx,
								   &saa716x_adap->fe_hw)) < 0) {

			dprintk(SAA716x_ERROR, 1, "dvb_dmx_init failed, ERROR=%d", result);
			goto err2;
		}

		saa716x_adap->fe_mem.source = DMX_MEMORY_FE;

		if ((result = saa716x_adap->demux.dmx.add_frontend(&saa716x_adap->demux.dmx,
								   &saa716x_adap->fe_mem)) < 0) {
			dprintk(SAA716x_ERROR, 1, "dvb_dmx_init failed, ERROR=%d", result);
			goto err3;
		}

		if ((result = saa716x_adap->demux.dmx.connect_frontend(&saa716x_adap->demux.dmx,
								       &saa716x_adap->fe_hw)) < 0) {

			dprintk(SAA716x_ERROR, 1, "dvb_dmx_init failed, ERROR=%d", result);
			goto err4;
		}

		dvb_net_init(&saa716x_adap->dvb_adapter, &saa716x_adap->dvb_net, &saa716x_adap->demux.dmx);
//		tasklet_init(&saa716x_adap->tasklet, saa716x_dma_xfer, (unsigned long) saa716x);
		dprintk(SAA716x_DEBUG, 1, "Frontend Init");
		saa716x_adap->saa716x = saa716x;

		if ((result = saa716x_frontend_power(saa716x, i, 1)) < 0) {
			dprintk(SAA716x_ERROR, 1, "SAA716x frontend powerup failed");
			goto err5;
		}

		if ((result = saa716x_frontend_reset(saa716x, i)) < 0) {
			dprintk(SAA716x_ERROR, 1, "SAA716x frontend reset failed");
			goto err5;
		}

		if (config->frontend_attach)
			config->frontend_attach(saa716x_adap, i);
		else
			dprintk(SAA716x_ERROR, 1, "Frontend attach = NULL");

		saa716x_adap++;
	}

	return 0;

	/* Error conditions */
err5:
	saa716x_frontend_power(saa716x, i, 0);
err4:
	saa716x_adap->demux.dmx.remove_frontend(&saa716x_adap->demux.dmx, &saa716x_adap->fe_mem);
err3:
	saa716x_adap->demux.dmx.remove_frontend(&saa716x_adap->demux.dmx, &saa716x_adap->fe_hw);
err2:
	dvb_dmxdev_release(&saa716x_adap->dmxdev);
err1:
	dvb_dmx_release(&saa716x_adap->demux);
err0:
	dvb_unregister_adapter(&saa716x_adap->dvb_adapter);

	return result;
}
EXPORT_SYMBOL(saa716x_dvb_init);

void __devexit saa716x_dvb_exit(struct saa716x_dev *saa716x)
{
	struct saa716x_adapter *saa716x_adap = saa716x->saa716x_adap;
	int i;

	for (i = 0; i < saa716x->config->adapters; i++) {

//		tasklet_kill(&saa716x->tasklet);
		dvb_net_release(&saa716x_adap->dvb_net);
		saa716x_adap->demux.dmx.remove_frontend(&saa716x_adap->demux.dmx, &saa716x_adap->fe_mem);
		saa716x_adap->demux.dmx.remove_frontend(&saa716x_adap->demux.dmx, &saa716x_adap->fe_hw);
		dvb_dmxdev_release(&saa716x_adap->dmxdev);
		dvb_dmx_release(&saa716x_adap->demux);

		if (saa716x_adap->fe)
			dvb_unregister_frontend(saa716x_adap->fe);

		dprintk(SAA716x_DEBUG, 1, "dvb_unregister_adapter");
		dvb_unregister_adapter(&saa716x_adap->dvb_adapter);

		saa716x_adap++;
	}

	return;
}
EXPORT_SYMBOL(saa716x_dvb_exit);
