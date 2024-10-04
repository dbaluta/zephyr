/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include <zephyr/cache.h>
#include "dma_nxp_sdma.h"

extern struct tr_ctx sdma_tr;

#define DMA_NXP_SDMA_BD_COUNT 2

#define DT_DRV_COMPAT nxp_sdma

#define DEV_CONFIG(dev) ((const struct sdma_dev_cfg *)dev->config)
#define DEV_BASE(dev) ((SDMAARM_Type *)DEV_CONFIG(dev)->base)
#define DEV_DATA(dev) ((struct sdma_dev_data *)dev->data)
#define DEV_CHANNEL_DATA(dev, c) ((struct sdma_channel_data *)(&DEV_DATA(dev)->chan[c]))
#define DEV_SDMA_HANDLE(dev, c) ((sdma_handle_t *)(&(DEV_CHANNEL_DATA(dev, c)->handle)))

#define SDMA_MAX_BD	2

void do_log(char const *p) 
{
}

AT_NONCACHEABLE_SECTION_ALIGN(static sdma_context_data_t sdma_contexts[2], 4);

void dma_nxp_sdma_callback(sdma_handle_t *handle, void *userData, bool transferDone,
			    uint32_t bdIndex);

struct sdma_dev_cfg {
	SDMAARM_Type *base;
	void (*irq_config)(void);
};

struct sdma_channel_data {
	sdma_handle_t handle;
	sdma_transfer_config_t transfer_cfg;
	sdma_peripheral_t peripheral;
	struct dma_block_config *crt_block; /* current block to be transferred */
	struct dma_config *dma_cfg;
	uint32_t event_source; /* DMA REQ numbers that trigger this channel */
};

struct sdma_dev_data {
	struct sdma_channel_data chan[32];
	sdma_buffer_descriptor_t bd_pool[32][DMA_NXP_SDMA_BD_COUNT] __aligned(64);
};

void *dma_nxp_sdma_get_base(const struct device *dev)
{
	return ((struct sdma_dev_cfg*)dev->config)->base;
}

#if 0
static void sdma_isr(const void *parameter)
{

}
#endif

void sdma_set_transfer_type(struct dma_config *config, sdma_transfer_type_t *type)
{
	switch (config->channel_direction) {
		case MEMORY_TO_MEMORY:
			*type = kSDMA_MemoryToMemory;
			break;
		case MEMORY_TO_PERIPHERAL:
			*type = kSDMA_MemoryToPeripheral;
			break;
		case PERIPHERAL_TO_MEMORY:
			*type = kSDMA_PeripheralToMemory;
			break;
		case PERIPHERAL_TO_PERIPHERAL:
			*type = kSDMA_PeripheralToPeripheral;
			break;
		default:
			LOG_ERR("channel direction not supported%d\n", config->channel_direction);
			return;
	}
	LOG_INF("sdma_set_transfer type DIR %d type = %d", config->channel_direction, *type);
}

void sdma_set_peripheral_type(struct dma_config *config, sdma_peripheral_t *type)
{
	*type = kSDMA_PeripheralNormal_SP;
}

void my_sys_cache_data_flush_all(void){
	sys_cache_data_flush_all();
}

/*static */int dma_nxp_sdma_channel_get(const struct device *dev, uint32_t channel)

{
	struct sdma_dev_data *dev_data = dev->data;
	struct sdma_channel_data *chan_data;
	sdma_handle_t *handle;

	chan_data = &dev_data->chan[channel];
	SDMA_CreateHandle(&chan_data->handle, DEV_BASE(dev), channel, &sdma_contexts[channel]);
	sys_cache_data_flush_all();

	LOG_INF("Created handle for chan %d %x sizeof(contexts) %d addr %08x %08x\n",
		channel, (int)&chan_data->handle, sizeof(sdma_contexts[channel]),
		&sdma_contexts[channel], chan_data->handle.context);

	return 0;
}

/*static*/ int dma_nxp_sdma_config(const struct device *dev, uint32_t channel,
		       struct dma_config *config)
{
	const struct sdma_dev_cfg *dev_cfg = dev->config;
	struct sdma_dev_data *dev_data = dev->data;
	struct sdma_channel_data *chan_data;

	/* TODO: verify channel index is valid */
	if (channel >= FSL_FEATURE_SDMA_MODULE_CHANNEL) {
		LOG_ERR("sdma_config() invalid channel %d", channel);
		return -EINVAL;
	}

	chan_data = &dev_data->chan[channel];

	sdma_set_transfer_type(config, &chan_data->transfer_cfg.type);
	sdma_set_peripheral_type(config, &chan_data->peripheral);

	chan_data->crt_block = config->head_block;
	chan_data->dma_cfg = config;

	/*TODO read this from dts file */
	chan_data->event_source = 5; // this is for playback, use 0 for
				     // apture
	/* prepare first block for transfer ...*/
	SDMA_PrepareTransfer(&chan_data->transfer_cfg,
			     chan_data->crt_block->source_address,
			     chan_data->crt_block->dest_address,
			     config->source_data_size, config->dest_data_size,
			     /* watermark = */64,
			     chan_data->crt_block->block_size, chan_data->event_source,
			     chan_data->peripheral, chan_data->transfer_cfg.type);
	
	/* ... and submit it to SDMA engine.
	 * Note that SDMA transfer is later manually started by the SDMA
	 * driver */

	chan_data->transfer_cfg.isEventIgnore = false;
	chan_data->transfer_cfg.isSoftTriggerIgnore = false;
	SDMA_SubmitTransfer(&chan_data->handle, &chan_data->transfer_cfg);

	LOG_INF("CONFIG: ch %d context %08x", channel, chan_data->handle.context);
	sys_cache_data_flush_all();
	return 0;
}


/* static */int dma_nxp_sdma_get_status(const struct device *dev, uint32_t chan_id,
			   struct dma_status *stat)
{
	int i;


	return 0;
}

/*static*/ int dma_nxp_sdma_stop(const struct device *dev, uint32_t channel)
{
	SDMA_StopTransfer(DEV_SDMA_HANDLE(dev, channel));
	return 0;
}

/*static*/ int dma_nxp_sdma_start(const struct device *dev, uint32_t channel)
{
	const struct sdma_dev_cfg *dev_cfg = dev->config;
	struct sdma_dev_data *dev_data = dev->data;
	struct sdma_channel_data *chan_data;
	sdma_handle_t *handle;


	chan_data = &dev_data->chan[channel];
	LOG_INF("SDMA start for chan %d handle %x\n", channel, (int)&chan_data->handle);

	SDMA_SetChannelPriority(dev_cfg->base, channel, 4);
	SDMA_StartChannelSoftware(dev_cfg->base, channel);
	
	//SDMA_StartTransfer(DEV_SDMA_HANDLE(dev, channel));
	return 0;
}


 int dma_nxp_sdma_reload(const struct device *dev, uint32_t channel, uint32_t src,
		       uint32_t dst, size_t size)
{
	const struct sdma_dev_cfg *dev_cfg = dev->config;
	struct sdma_dev_data *dev_data = dev->data;
	struct sdma_channel_data *chan_data;

	chan_data = &dev_data->chan[channel];
	chan_data->crt_block = chan_data->crt_block->next_block;

	LOG_INF("Sending next block %x dest %x",
			     chan_data->crt_block->source_address,
			     chan_data->crt_block->dest_address);

	SDMA_PrepareTransfer(&chan_data->transfer_cfg,
			     chan_data->crt_block->source_address,
			     chan_data->crt_block->dest_address,
			     chan_data->dma_cfg->source_data_size,
			     chan_data->dma_cfg->dest_data_size,
			     /* watermark = */64,
			     chan_data->crt_block->block_size, chan_data->event_source,
			     chan_data->peripheral, chan_data->transfer_cfg.type);

	chan_data->transfer_cfg.isEventIgnore = false;
	chan_data->transfer_cfg.isSoftTriggerIgnore = false;

	SDMA_SubmitTransfer(&chan_data->handle, &chan_data->transfer_cfg);

	sys_cache_data_flush_all();
	return 0;
}

static int dma_nxp_sdma_get_attribute(const struct device *dev, uint32_t type, uint32_t *val)
{
	switch(type) {
		case DMA_ATTR_BUFFER_SIZE_ALIGNMENT:
			*val = 4;
			break;
		case DMA_ATTR_BUFFER_ADDRESS_ALIGNMENT:
			*val = 128; /* should be dcache_align */
			break;
		case DMA_ATTR_MAX_BLOCK_COUNT:
			*val = CONFIG_DMA_NXP_SDMA_BD_COUNT;
			break;
		default:
			LOG_ERR("invalid attribute type: %d", type);
			return -EINVAL;
	}
	return 0;
}

#if 0
static bool sdma_channel_filter(const struct device *dev, int chan_id, void *param)
{
	return false;
}
#endif

static const struct dma_driver_api sdma_api = {
	.reload = dma_nxp_sdma_reload,
	.config = dma_nxp_sdma_config,
	.start = dma_nxp_sdma_start,
	.stop = dma_nxp_sdma_stop,
	.suspend = dma_nxp_sdma_stop,
	.resume = dma_nxp_sdma_start,
	.get_status = dma_nxp_sdma_get_status,
	.get_attribute = dma_nxp_sdma_get_attribute,
//	.chan_filter = sdma_channel_filter,
};

void dma_nxp_sdma_print_regs(const struct device *dev, const char *str)
{
	SDMAARM_Type *base = dma_nxp_sdma_get_base(dev);
	LOG_INF(" === %s === ", str);
	LOG_INF("MCOPTR: %08x", base->MC0PTR);
	LOG_INF("INTR: %08x", base->INTR);
	LOG_INF("STOP_STAT: %08x", base->STOP_STAT);
	LOG_INF("HSTART: %08x", base->HSTART);
	LOG_INF("EVTOVR: %08x", base->EVTOVR);
	LOG_INF("DSPOVR: %08x", base->DSPOVR);
	LOG_INF("HOSTOVR: %08x", base->HOSTOVR);

	LOG_INF("EVTPEND: %08x", base->EVTPEND);
	LOG_INF("INTRMASK: %08x", base->INTRMASK);
	LOG_INF("CONFIG: %08x", base->CONFIG);
	LOG_INF("CHN0ADDR: %08x", base->CHN0ADDR);
	LOG_INF("EVTMIRROR %08x", base->EVT_MIRROR);
	LOG_INF("CHANRPI[0]: %08x", base->SDMA_CHNPRI[0]);
	LOG_INF("CHANRPI[1]: %08x", base->SDMA_CHNPRI[1]);
	LOG_INF("CHNENBL[0]: %08x", base->CHNENBL[0]);
	LOG_INF("CHNENBL[5]: %08x", base->CHNENBL[5]);
}

void dma_nxp_sdma_print_ccb(sdma_channel_control_t *ccb, int i)
{
	sdma_buffer_descriptor_t *bd;

	LOG_INF("== CCB %08x %d", ccb, i);
	LOG_INF("  crt_bd %08x", ccb->currentBDAddr);
	LOG_INF("  base  %08x", ccb->baseBDAddr);
	LOG_INF("  channelDesc %08x", ccb->channelDesc);
	LOG_INF("  channelStatus %08x", ccb->status);

	bd = (sdma_buffer_descriptor_t *) ccb->currentBDAddr;
	LOG_INF("CMD: %08x STATUS L %d, ER %d I %d C %d D %d W %d count %d", bd->command,
		bd->status & kSDMA_BDStatusLast ? 1 : 0,
		bd->status & kSDMA_BDStatusError ? 1 : 0,
		bd->status & kSDMA_BDStatusInterrupt? 1 : 0,
		bd->status & kSDMA_BDStatusContinuous? 1 : 0,
		bd->status & kSDMA_BDStatusDone? 1 : 0,
		bd->status & kSDMA_BDStatusWrap? 1 : 0,
		bd->count);
	LOG_INF("BD addr %08x", bd->bufferAddr);
}

void dma_nxp_sdma_dump_info(const struct device *dev, const char *str)
{
	SDMAARM_Type *base = dma_nxp_sdma_get_base(dev);
	sdma_channel_control_t *ccb0 = base->MC0PTR;
	sdma_channel_control_t *ccb1 = ccb0;

	dma_nxp_sdma_print_ccb(ccb0, 0);
	ccb1++;
	dma_nxp_sdma_print_ccb(ccb1, 1);
}

void dma_nxp_sdma_print_context(const struct device *dev, int chan, 
				void *ctx, const char *str)
{

	struct sdma_dev_data *dev_data = dev->data;
	struct sdma_channel_data *chan_data;
	sdma_handle_t *handle;

	chan_data = &dev_data->chan[chan];
	sdma_context_data_t *context;

	if (!ctx)
		context = chan_data->handle.context;
	else
		context = ctx;

	LOG_INF(" === %s ==", str);
	LOG_INF("PC %08x", context->PC);
	LOG_INF("GREG[0] %08x GREG[1] %08x", context->GeneralReg[0], context->GeneralReg[1]);
	LOG_INF("GREG[6] %08x GREG[7] %08x", context->GeneralReg[6], context->GeneralReg[7]);
}

/*static*/ int dma_nxp_sdma_init(const struct device *dev)
{
	const struct sdma_dev_cfg *cfg = dev->config;
	sdma_config_t defconfig;

	SDMA_GetDefaultConfig(&defconfig);
	defconfig.ratio = kSDMA_ARMClockFreq;

	SDMA_Init(cfg->base, &defconfig);
	sys_cache_data_flush_all();
	dma_nxp_sdma_print_regs(dev, "after init");
	LOG_INF("dma_nxp_sdma_init");

	return 0;
	/* configure interrupts */

	cfg->irq_config();
	/*TODO: boot and load new firmeware */
	return 0;
}


#if 0
void dma_nxp_do_copy(struct device *dev, int channel, int bdIndex)
{
	sdma_handle_t *handle;
	struct sdma_channel *sdma_chan;

	sdma_chan = DEV_CHANNEL_DATA(dev, channel);
	handle = DEV_SDMA_HANDLE(dev, channel);

	dma_nxp_sdma_callback(handle, sdma_chan, true, bdIndex);
}
#endif
void dma_nxp_sdma_callback(sdma_handle_t *handle, void *userData, bool transferDone,

			    uint32_t bdIndex)
{
#if 0
	struct sdma_channel *sdma_chan = userData;
	sdma_buffer_descriptor_t *currentBD = NULL;//BDPool[bdIndex];

	/* mark current BD as ready for transfer */
	currentBD->status |= kSDMA_BDStatusDone;

	sdma_chan->status.read_position = currentBD->bufferAddr;
	sdma_chan->status.pending_length = currentBD->count;
#endif
}

static void dma_nxp_sdma_isr(const struct device *dev)
{
	SDMA_DriverIRQHandler();
}

#define SDMA_INIT(inst)						               \
static struct sdma_dev_data sdma_data_##inst; 				  	\
static const struct sdma_dev_cfg sdma_cfg_##inst = {				\
	.base = DT_INST_REG_ADDR(inst),						\
};										\
										\
DEVICE_DT_INST_DEFINE(inst, &dma_nxp_sdma_init, NULL,				\
		      &sdma_data_##inst, &sdma_cfg_##inst,			\
		      PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,			\
		      &sdma_api);						\

DT_INST_FOREACH_STATUS_OKAY(SDMA_INIT);
