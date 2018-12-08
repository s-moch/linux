#include <linux/bitops.h>

#include <media/dmxdev.h>
#include <media/dvbdev.h>
#include <media/dvb_demux.h>
#include <media/dvb_frontend.h>

#include "saa716x_greg_reg.h"

#include "saa716x_mod.h"
#include "saa716x_adap.h"
#include "saa716x_i2c.h"
#include "saa716x_priv.h"


#define SAA716X_TS_DMA_BUF_SIZE		(16 * SAA716x_PAGE_SIZE)

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);


void saa716x_dma_start(struct saa716x_dev *saa716x, u8 adapter)
{
	struct fgpi_stream_params params;

	dprintk(SAA716x_DEBUG, 1, "SAA716x Start DMA engine for Adapter:%d", adapter);

	params.bits		= 8;
	params.samples		= 188;
	params.lines		= SAA716X_TS_DMA_BUF_SIZE / 188;
	params.pitch		= 188;
	params.offset		= 0;
	params.page_tables	= 0;
	params.stream_type	= FGPI_TRANSPORT_STREAM;
	params.stream_flags	= 0;

	saa716x_fgpi_start(saa716x, saa716x->config->adap_config[adapter].ts_fgpi, &params);
}

void saa716x_dma_stop(struct saa716x_dev *saa716x, u8 adapter)
{
	dprintk(SAA716x_DEBUG, 1, "SAA716x Stop DMA engine for Adapter:%d", adapter);

	saa716x_fgpi_stop(saa716x, saa716x->config->adap_config[adapter].ts_fgpi);
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
		saa716x_dma_start(saa716x, saa716x_adap->count);
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
		saa716x_dma_stop(saa716x, saa716x_adap->count);
	}

	return 0;
}

static void saa716x_demux_worker(unsigned long data)
{
	struct saa716x_fgpi_stream_port *fgpi_entry = (struct saa716x_fgpi_stream_port *)data;
	struct saa716x_dev *saa716x = fgpi_entry->saa716x;
	struct dvb_demux *demux;
	u32 fgpi_index;
	u32 i;
	u32 write_index;

	fgpi_index = fgpi_entry->dma_channel - 6;
	demux = NULL;
	for (i = 0; i < saa716x->config->adapters; i++) {
		if (saa716x->config->adap_config[i].ts_fgpi == fgpi_index) {
			demux = &saa716x->saa716x_adap[i].demux;
			break;
		}
	}
	if (demux == NULL) {
		printk(KERN_ERR "%s: unexpected channel %u\n",
		       __func__, fgpi_entry->dma_channel);
		return;
	}

	write_index = saa716x_fgpi_get_write_index(saa716x, fgpi_index);
	if (write_index < 0)
		return;

	dprintk(SAA716x_DEBUG, 1, "dma buffer = %d", write_index);

	if (write_index == fgpi_entry->read_index) {
		printk(KERN_DEBUG "%s: called but nothing to do\n", __func__);
		return;
	}

	do {
		u8 *data = (u8 *)fgpi_entry->dma_buf[fgpi_entry->read_index].mem_virt;

		pci_dma_sync_sg_for_cpu(saa716x->pdev,
			fgpi_entry->dma_buf[fgpi_entry->read_index].sg_list,
			fgpi_entry->dma_buf[fgpi_entry->read_index].list_len,
			PCI_DMA_FROMDEVICE);

		dvb_dmx_swfilter(demux, data, 348 * 188);

		fgpi_entry->read_index = (fgpi_entry->read_index + 1) & 7;
	} while (write_index != fgpi_entry->read_index);
}

int saa716x_dvb_init(struct saa716x_dev *saa716x)
{
	struct saa716x_adapter *saa716x_adap = saa716x->saa716x_adap;
	struct saa716x_config *config = saa716x->config;
	int result, i;

	mutex_init(&saa716x->adap_lock);

	/* all video input ports use their own clocks */
	SAA716x_EPWR(GREG, GREG_VI_CTRL, 0x2C688000);
	SAA716x_EPWR(GREG, GREG_FGPI_CTRL, 0);

	for (i = 0; i < config->adapters; i++) {

		dprintk(SAA716x_DEBUG, 1, "dvb_register_adapter");
		if (dvb_register_adapter(&saa716x_adap->dvb_adapter,
					 "SAA716x dvb adapter",
					 saa716x->module,
					 &saa716x->pdev->dev,
					 adapter_nr) < 0) {

			dprintk(SAA716x_ERROR, 1, "Error registering adapter");
			return -ENODEV;
		}

		saa716x_adap->count			= i;

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

		if (config->frontend_attach) {
			result = config->frontend_attach(saa716x_adap, i);
			if (result < 0)
				dprintk(SAA716x_ERROR, 1, "SAA716x frontend attach failed");

			if (saa716x_adap->fe == NULL) {
				dprintk(SAA716x_ERROR, 1, "A frontend driver was not found for [%04x:%04x] subsystem [%04x:%04x]\n",
					saa716x->pdev->vendor,
					saa716x->pdev->device,
					saa716x->pdev->subsystem_vendor,
					saa716x->pdev->subsystem_device);
			} else {
				result = dvb_register_frontend(&saa716x_adap->dvb_adapter, saa716x_adap->fe);
				if (result < 0) {
					dprintk(SAA716x_ERROR, 1, "SAA716x register frontend failed");
					goto err6;
				}
			}

		} else {
			dprintk(SAA716x_ERROR, 1, "Frontend attach = NULL");
		}

		/* assign video port to fgpi */
		SAA716x_EPWR(GREG, GREG_FGPI_CTRL, SAA716x_EPRD(GREG, GREG_FGPI_CTRL) |
		             (GREG_FGPI_CTRL_SEL(config->adap_config[i].ts_vp) << (config->adap_config[i].ts_fgpi * 3)));

		saa716x_fgpi_init(saa716x, config->adap_config[i].ts_fgpi,
				  SAA716X_TS_DMA_BUF_SIZE,
				  saa716x_demux_worker);

		saa716x_adap++;
	}


	return 0;

	/* Error conditions */
err6:
	dvb_frontend_detach(saa716x_adap->fe);
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

void saa716x_dvb_exit(struct saa716x_dev *saa716x)
{
	struct saa716x_adapter *saa716x_adap = saa716x->saa716x_adap;
	struct i2c_client *client;
	int i;

	for (i = 0; i < saa716x->config->adapters; i++) {

		saa716x_fgpi_exit(saa716x, saa716x->config->adap_config[i].ts_fgpi);

		/* remove I2C tuner */
		client = saa716x_adap->i2c_client_tuner;
		if (client) {
			module_put(client->dev.driver->owner);
			i2c_unregister_device(client);
		}

		/* remove I2C demod */
		client = saa716x_adap->i2c_client_demod;
		if (client) {
			module_put(client->dev.driver->owner);
			i2c_unregister_device(client);
		}

		if (saa716x_adap->fe) {
			dvb_unregister_frontend(saa716x_adap->fe);
			dvb_frontend_detach(saa716x_adap->fe);
		}

//		tasklet_kill(&saa716x->tasklet);
		dvb_net_release(&saa716x_adap->dvb_net);
		saa716x_adap->demux.dmx.remove_frontend(&saa716x_adap->demux.dmx, &saa716x_adap->fe_mem);
		saa716x_adap->demux.dmx.remove_frontend(&saa716x_adap->demux.dmx, &saa716x_adap->fe_hw);
		dvb_dmxdev_release(&saa716x_adap->dmxdev);
		dvb_dmx_release(&saa716x_adap->demux);

		dprintk(SAA716x_DEBUG, 1, "dvb_unregister_adapter");
		dvb_unregister_adapter(&saa716x_adap->dvb_adapter);

		saa716x_adap++;
	}

	return;
}
EXPORT_SYMBOL(saa716x_dvb_exit);
