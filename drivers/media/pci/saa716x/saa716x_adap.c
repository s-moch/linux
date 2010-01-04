#include "saa716x_priv.h"

#include <linux/bitops.h>

#include "dmxdev.h"
#include "dvbdev.h"
#include "dvb_demux.h"
#include "dvb_frontend.h"

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
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct saa716x_dev *saa716x = dvbdmx->priv;

	dprintk(SAA716x_DEBUG, 1, "SAA716x DVB Start feed");
	if (!dvbdmx->dmx.frontend) {
		dprintk(SAA716x_DEBUG, 1, "no frontend ?");
		return -EINVAL;
	}
	saa716x->feeds++;
	dprintk(SAA716x_DEBUG, 1, "SAA716x start feed, feeds=%d",
		saa716x->feeds);

	if (saa716x->feeds == 1) {
		dprintk(SAA716x_DEBUG, 1, "SAA716x start feed & dma");
		printk("saa716x start feed & dma\n");
		saa716x_dma_start(saa716x);
	}

	return saa716x->feeds;
}

static int saa716x_dvb_stop_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct saa716x_dev *saa716x = dvbdmx->priv;

	dprintk(SAA716x_DEBUG, 1, "SAA716x DVB Stop feed");
	if (!dvbdmx->dmx.frontend) {
		dprintk(SAA716x_DEBUG, 1, "no frontend ?");
		return -EINVAL;
	}
	saa716x->feeds--;
	if (saa716x->feeds == 0) {
		dprintk(SAA716x_DEBUG, 1, "saa716x stop feed and dma");
		printk("saa716x stop feed and dma\n");
		saa716x_dma_stop(saa716x);
	}
	return 0;
}

int __devinit saa716x_frontend_init(struct saa716x_dev *saa716x)
{
	dprintk(SAA716x_DEBUG, 1, "SAA716x frontend Init");
	dprintk(SAA716x_DEBUG, 1, "Device ID=%02x", saa716x->pdev->subsystem_device);
	switch (saa716x->pdev->subsystem_device) {
	default:
		dprintk(SAA716x_DEBUG, 1, "Unknown frontend:[0x%02x]",
			saa716x->pdev->subsystem_device);

		return -ENODEV;
	}
	if (saa716x->fe == NULL) {
		dprintk(SAA716x_ERROR, 1, "!!! NO Frontends found !!!");
		return -ENODEV;
	} else {
		if (dvb_register_frontend(&saa716x->dvb_adapter, saa716x->fe)) {
			dprintk(SAA716x_ERROR, 1, "ERROR: Frontend registration failed");

			if (saa716x->fe->ops.release)
				saa716x->fe->ops.release(saa716x->fe);

			saa716x->fe = NULL;
			return -ENODEV;
		}
	}

	return 0;
}

int __devinit saa716x_dvb_init(struct saa716x_dev *saa716x)
{
	int result;

	dprintk(SAA716x_DEBUG, 1, "dvb_register_adapter");
	if (dvb_register_adapter(&saa716x->dvb_adapter,
				 "SAA716x dvb adapter", THIS_MODULE,
				 &saa716x->pdev->dev) < 0) {

		dprintk(SAA716x_ERROR, 1, "Error registering adapter");
		return -ENODEV;
	}
	saa716x->dvb_adapter.priv = saa716x;
	saa716x->demux.dmx.capabilities = DMX_TS_FILTERING	|
					  DMX_SECTION_FILTERING	|
					  DMX_MEMORY_BASED_FILTERING;

	saa716x->demux.priv = saa716x;
	saa716x->demux.filternum = 256;
	saa716x->demux.feednum = 256;
	saa716x->demux.start_feed = saa716x_dvb_start_feed;
	saa716x->demux.stop_feed = saa716x_dvb_stop_feed;
	saa716x->demux.write_to_decoder = NULL;
	dprintk(SAA716x_DEBUG, 1, "dvb_dmx_init");
	if ((result = dvb_dmx_init(&saa716x->demux)) < 0) {
		dprintk(SAA716x_ERROR, 1, "dvb_dmx_init failed, ERROR=%d", result);
		goto err0;
	}
	saa716x->dmxdev.filternum = 256;
	saa716x->dmxdev.demux = &saa716x->demux.dmx;
	saa716x->dmxdev.capabilities = 0;
	dprintk(SAA716x_DEBUG, 1, "dvb_dmxdev_init");
	if ((result = dvb_dmxdev_init(&saa716x->dmxdev,
				      &saa716x->dvb_adapter)) < 0) {

		dprintk(SAA716x_ERROR, 1, "dvb_dmxdev_init failed, ERROR=%d", result);
		goto err1;
	}
	saa716x->fe_hw.source = DMX_FRONTEND_0;
	if ((result = saa716x->demux.dmx.add_frontend(&saa716x->demux.dmx,
						      &saa716x->fe_hw)) < 0) {

		dprintk(SAA716x_ERROR, 1, "dvb_dmx_init failed, ERROR=%d", result);
		goto err2;
	}
	saa716x->fe_mem.source = DMX_MEMORY_FE;
	if ((result = saa716x->demux.dmx.add_frontend(&saa716x->demux.dmx,
						      &saa716x->fe_mem)) < 0) {
		dprintk(SAA716x_ERROR, 1, "dvb_dmx_init failed, ERROR=%d", result);
		goto err3;
	}
	if ((result = saa716x->demux.dmx.connect_frontend(&saa716x->demux.dmx,
							  &saa716x->fe_hw)) < 0) {

		dprintk(SAA716x_ERROR, 1, "dvb_dmx_init failed, ERROR=%d", result);
		goto err4;
	}
	dvb_net_init(&saa716x->dvb_adapter, &saa716x->dvb_net, &saa716x->demux.dmx);
//	tasklet_init(&saa716x->tasklet, saa716x_dma_xfer, (unsigned long) saa716x);
	saa716x_frontend_init(saa716x);

	return 0;

	/*	Error conditions ..	*/
err4:
	saa716x->demux.dmx.remove_frontend(&saa716x->demux.dmx, &saa716x->fe_mem);
err3:
	saa716x->demux.dmx.remove_frontend(&saa716x->demux.dmx, &saa716x->fe_hw);
err2:
	dvb_dmxdev_release(&saa716x->dmxdev);
err1:
	dvb_dmx_release(&saa716x->demux);
err0:
	dvb_unregister_adapter(&saa716x->dvb_adapter);

	return result;
}

void __devexit saa716x_dvb_exit(struct saa716x_dev *saa716x)
{
//	tasklet_kill(&saa716x->tasklet);
	dvb_net_release(&saa716x->dvb_net);
	saa716x->demux.dmx.remove_frontend(&saa716x->demux.dmx, &saa716x->fe_mem);
	saa716x->demux.dmx.remove_frontend(&saa716x->demux.dmx, &saa716x->fe_hw);
	dvb_dmxdev_release(&saa716x->dmxdev);
	dvb_dmx_release(&saa716x->demux);

	if (saa716x->fe)
		dvb_unregister_frontend(saa716x->fe);
	dprintk(SAA716x_DEBUG, 1, "dvb_unregister_adapter");
	dvb_unregister_adapter(&saa716x->dvb_adapter);

	return;
}
