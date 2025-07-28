/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include <zephyr/cache.h>
#include <zephyr/logging/log.h>
#include "fsl_sdma.h"

LOG_MODULE_REGISTER(nxp_sdma);



#define DMA_NXP_SDMA_BD_COUNT 2
#define DMA_NXP_SDMA_CHAN_DEFAULT_PRIO 4

#define DT_DRV_COMPAT nxp_sdma

#if 0
AT_NONCACHEABLE_SECTION_ALIGN(static sdma_context_data_t
			      sdma_contexts[FSL_FEATURE_SDMA_MODULE_CHANNEL], 4);
#endif

static sdma_context_data_t __nocache sdma_contexts[FSL_FEATURE_SDMA_MODULE_CHANNEL] __aligned(CONFIG_DCACHE_LINE_SIZE);

struct sdma_dev_cfg {
	SDMAARM_Type *base;
	void (*irq_config)(void);
};

struct sdma_channel_data {
	sdma_handle_t handle;
	sdma_transfer_config_t transfer_cfg;
	sdma_peripheral_t peripheral;
	uint32_t direction;
	uint32_t index;
	const struct device *dev;
	sdma_buffer_descriptor_t *bd_pool; /*pre-allocated list of BD used for transfer */
	uint32_t bd_count; /* number of bd */
	uint32_t capacity; /* total transfer capacity for this channel */
	struct dma_config *dma_cfg;
	uint32_t event_source; /* DMA REQ number that trigger this channel */
	struct dma_status stat;

	void *arg; /* argument passed to user-defined DMA callback */
	dma_callback_t cb; /* user-defined callback for DMA transfer completion */
};

struct sdma_dev_data {
	struct dma_context dma_ctx;
	atomic_t *channels_atomic;
	struct sdma_channel_data chan[FSL_FEATURE_SDMA_MODULE_CHANNEL];
	sdma_buffer_descriptor_t bd_pool[FSL_FEATURE_SDMA_MODULE_CHANNEL][DMA_NXP_SDMA_BD_COUNT]
		__aligned(64);
};

static inline void sdma_dump_register_data(SDMAARM_Type *base)
{

	LOG_INF("MCOPTR    :%08x\n", base->MC0PTR);
	LOG_INF("INTR      :%08x\n", base->INTR);
	LOG_INF("STOP_STAT :%08x\n", base->STOP_STAT);
	LOG_INF("HSTART    :%08x\n", base->HSTART);
	LOG_INF("EVTOVR    :%08x\n", base->EVTOVR);
	LOG_INF("HOSTOVR   :%08x\n", base->HOSTOVR);
	LOG_INF("EVTPD     :%08x\n", base->EVTPEND);
	LOG_INF("EVTERRDBG :%08x\n", base->EVTERRDBG);
	LOG_INF("EVTERR    :%08x\n", base->EVTERR);
	LOG_INF("PSW	   :%08x\n", base->PSW);
	LOG_INF("INTRMASK  :%08x\n", base->INTRMASK);
	LOG_INF("CONFIG    :%08x\n", base->CONFIG);
	LOG_INF("CHNDADDR  :%08x\n", base->CHN0ADDR);
	LOG_INF("EVT_MRR   :%08x\n", base->EVT_MIRROR);
	LOG_INF("CHNPRI    :%08x\n", base->SDMA_CHNPRI[1]);
	LOG_INF("CHNENBL[4]:%08x\n", base->CHNENBL[4]);
	LOG_INF("CHNENBL[5]:%08x\n", base->CHNENBL[5]);
}


static int dma_nxp_sdma_init_stat(struct sdma_channel_data *chan_data)
{
	chan_data->stat.read_position = 0;
	chan_data->stat.write_position = 0;

	switch (chan_data->direction) {
	case MEMORY_TO_PERIPHERAL:
		/* buffer is full */
		chan_data->stat.pending_length = chan_data->capacity;
		chan_data->stat.free = 0;
		break;
	case PERIPHERAL_TO_MEMORY:
		/* buffer is empty */
		chan_data->stat.pending_length = 0;
		chan_data->stat.free = chan_data->capacity;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int dma_nxp_sdma_consume(struct sdma_channel_data *chan_data, uint32_t bytes)
{
	if (bytes > chan_data->stat.pending_length)
		return -EINVAL;

	chan_data->stat.read_position += bytes;
	chan_data->stat.read_position %= chan_data->capacity;

	if (chan_data->stat.read_position > chan_data->stat.write_position) {
		chan_data->stat.free = chan_data->stat.read_position -
			chan_data->stat.write_position;
	} else {
		chan_data->stat.free = chan_data->capacity -
			(chan_data->stat.write_position - chan_data->stat.read_position);
	}

	chan_data->stat.pending_length = chan_data->capacity - chan_data->stat.free;

	return 0;
}

static int dma_nxp_sdma_produce(struct sdma_channel_data *chan_data, uint32_t bytes)
{
	if (bytes > chan_data->stat.free)
		return -EINVAL;

	chan_data->stat.write_position += bytes;
	chan_data->stat.write_position %= chan_data->capacity;

	if (chan_data->stat.write_position > chan_data->stat.read_position) {
		chan_data->stat.pending_length = chan_data->stat.write_position -
			chan_data->stat.read_position;
	} else {
		chan_data->stat.pending_length = chan_data->capacity -
			(chan_data->stat.read_position - chan_data->stat.write_position);
	}

	chan_data->stat.free = chan_data->capacity - chan_data->stat.pending_length;

	return 0;
}

static void dma_nxp_sdma_isr(const void *data)
{
	uint32_t val, val1;
	uint32_t i = 1;
	struct sdma_channel_data *chan_data;
	struct device *dev = (struct device *)data;
	struct sdma_dev_data *dev_data = dev->data;
	const struct sdma_dev_cfg *dev_cfg = dev->config;
	sdma_buffer_descriptor_t *bd;
	static int count1 = 0;


	val1 = SDMA_GetChannelInterruptStatus(dev_cfg->base);

	/* Clear channel 0 */
	SDMA_ClearChannelInterruptStatus(dev_cfg->base, 1U);

	/* Ignore channel 0, is used only for download */
	val = SDMA_GetChannelInterruptStatus(dev_cfg->base);
	

	if (count1 < 3)
		LOG_INF("dma_nxp_sdma_sir val %08x val1 %08x\n", val, val1);

	val = val >> 1U;

	while (val) {
		if ((val & 0x1) != 0) {
			chan_data = &dev_data->chan[i];

			sys_cache_data_invd_all();

			bd = &chan_data->bd_pool[0];
			if (count1 < 1) {
				//print_buffer_descriptor("interupt", 0, bd);
				//sdma_dump_register_data(dev_cfg->base);
			}

			count1++;
			SDMA_ClearChannelInterruptStatus(dev_cfg->base, 1 << i);
			SDMA_HandleIRQ(&chan_data->handle);

			if (chan_data->cb)
				chan_data->cb(chan_data->dev, chan_data->arg, i, DMA_STATUS_BLOCK);
		}
		i++;
		val >>= 1;
	}
}

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
		LOG_ERR("%s: channel direction not supported %d", __func__,
			config->channel_direction);
		return;
	}
	LOG_DBG("%s: dir %d type = %d", __func__, config->channel_direction, *type);
}

int sdma_set_peripheral_type(struct dma_config *config, sdma_peripheral_t *type)
{
	switch (config->dma_slot) {
	case kSDMA_PeripheralNormal_SP:
	case kSDMA_PeripheralMultiFifoPDM:
		*type = config->dma_slot;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

void print_buffer_descriptor(const char *s, int i, sdma_buffer_descriptor_t *bd)
{
	LOG_INF("[%s] BD %d -> count %d status [D=%d W=%d C=%d I=%d E=%d L=%d],cmd %d buf_addr %08x ext_addr %08x\n", s, 
		i, bd->count,
		bd->status & kSDMA_BDStatusDone,
		bd->status & kSDMA_BDStatusWrap,
		bd->status & kSDMA_BDStatusContinuous,
		bd->status & kSDMA_BDStatusInterrupt,
		bd->status & kSDMA_BDStatusError,
		bd->status & kSDMA_BDStatusLast,
		
		bd->command, (int)bd->bufferAddr, (int)bd->extendBufferAddr);
}

void dma_nxp_sdma_callback(sdma_handle_t *handle, void *userData, bool TransferDone,
			   uint32_t bdIndex)
{
	const struct sdma_dev_cfg *dev_cfg;
	struct sdma_channel_data *chan_data = userData;
	sdma_buffer_descriptor_t *bd;
	int xfer_size;
	static int count_1 = 0;

	dev_cfg = chan_data->dev->config;

	xfer_size = chan_data->capacity / chan_data->bd_count;

	if (count_1 < 1)
		LOG_INF("Calling callback index %d xfer_size %d\n", bdIndex,
			xfer_size);
	count_1++;

	switch (chan_data->direction) {
	case MEMORY_TO_PERIPHERAL:
		dma_nxp_sdma_consume(chan_data, xfer_size);
		break;
	case PERIPHERAL_TO_MEMORY:
		dma_nxp_sdma_produce(chan_data, xfer_size);
		break;
	default:
		break;
	}

	sys_cache_data_invd_all();


	/* prepare next BD for transfer */
	bd = &chan_data->bd_pool[bdIndex];
	if (count_1 < 1)
		//print_buffer_descriptor("callback", bdIndex, bd);
	count_1++;

	bd->count = xfer_size;
	bd->status |= (uint8_t)kSDMA_BDStatusDone;

	sys_cache_data_flush_all();


	SDMA_StartChannelSoftware(dev_cfg->base, chan_data->index);
}

static int dma_nxp_sdma_channel_init(const struct device *dev, uint32_t channel)
{
	const struct sdma_dev_cfg *dev_cfg = dev->config;
	struct sdma_dev_data *dev_data = dev->data;
	struct sdma_channel_data *chan_data;

	chan_data = &dev_data->chan[channel];
	SDMA_CreateHandle(&chan_data->handle, dev_cfg->base, channel, &sdma_contexts[channel]);

	SDMA_SetCallback(&chan_data->handle, dma_nxp_sdma_callback, chan_data);
	
	SDMA_EnableChannelErrorInterrupts(dev_cfg->base, channel);

	sys_cache_data_flush_all();

	return 0;
}


static void dma_nxp_sdma_setup_bd(const struct device *dev, uint32_t channel,
				struct dma_config *config)
{
	struct sdma_dev_data *dev_data = dev->data;
	struct sdma_channel_data *chan_data;
	sdma_buffer_descriptor_t *crt_bd;
	struct dma_block_config *block_cfg;
	int i;

	chan_data = &dev_data->chan[channel];

	chan_data->capacity = 0;

	/* initialize bd pool */
	chan_data->bd_pool = &dev_data->bd_pool[channel][0];
	chan_data->bd_count = config->block_count;

	memset(chan_data->bd_pool, 0, sizeof(sdma_buffer_descriptor_t) * chan_data->bd_count);
	SDMA_InstallBDMemory(&chan_data->handle, chan_data->bd_pool, chan_data->bd_count);


	crt_bd = chan_data->bd_pool;
	block_cfg = config->head_block;

	for (i = 0; i < config->block_count; i++) {
		bool is_last = false;
		bool is_wrap = false;

		if (i == config->block_count - 1) {
			is_last = true;
			is_wrap = true;
		}

	
		LOG_INF("BD = %d size %d src %08x dst %08x", i,
			block_cfg->block_size,
			(int)block_cfg->source_address,
 			(int)block_cfg->dest_address);


		SDMA_ConfigBufferDescriptor(crt_bd,
			block_cfg->source_address, block_cfg->dest_address,
			config->source_data_size, block_cfg->block_size,
			is_last, true, is_wrap, chan_data->transfer_cfg.type);

		chan_data->capacity += block_cfg->block_size;
		block_cfg = block_cfg->next_block;

		print_buffer_descriptor("setup_bd", i, crt_bd);
		crt_bd++;
	}
	sys_cache_data_flush_all();
}

static int dma_nxp_sdma_config(const struct device *dev, uint32_t channel,
			       struct dma_config *config)
{
	const struct sdma_dev_cfg *dev_cfg = dev->config;
	struct sdma_dev_data *dev_data = dev->data;
	struct sdma_channel_data *chan_data;
	struct dma_block_config *block_cfg;
	int ret;

	if (channel >= FSL_FEATURE_SDMA_MODULE_CHANNEL) {
		LOG_ERR("sdma_config() invalid channel %d", channel);
		return -EINVAL;
	}

	dma_nxp_sdma_channel_init(dev, channel);

	chan_data = &dev_data->chan[channel];
	chan_data->dev = dev;
	chan_data->direction = config->channel_direction;

	chan_data->cb = config->dma_callback;
	chan_data->arg = config->user_data;

	sdma_set_transfer_type(config, &chan_data->transfer_cfg.type);

	ret = sdma_set_peripheral_type(config, &chan_data->peripheral);
	if (ret < 0) {
		LOG_ERR("%s: failed to set peripheral type", __func__);
		return ret;
	}

	if (chan_data->peripheral == kSDMA_PeripheralMultiFifoPDM) {
		unsigned int n_fifos = 4; /* TODO: make this configurable */

		SDMA_SetMultiFifoConfig(&chan_data->transfer_cfg, n_fifos, 0);
		SDMA_EnableSwDone(dev_cfg->base, &chan_data->transfer_cfg, 0,
				  chan_data->peripheral);
	}

	dma_nxp_sdma_setup_bd(dev, channel, config);
	ret = dma_nxp_sdma_init_stat(chan_data);
	if (ret < 0) {
		LOG_ERR("%s: failed to init stat", __func__);
		return ret;
	}

	block_cfg = config->head_block;

	/* prepare first block for transfer ...*/
	SDMA_PrepareTransfer(&chan_data->transfer_cfg,
			     block_cfg->source_address,
			     block_cfg->dest_address,
			     config->source_data_size, config->dest_data_size,
			     /* watermark = */ 64,
			     block_cfg->block_size, chan_data->event_source,
			     chan_data->peripheral, chan_data->transfer_cfg.type);

	/*... and submit it to SDMA engine.
	 * Note that SDMA transfer is later manually started by the dma_nxp_sdma_start()
	 */
	chan_data->transfer_cfg.isEventIgnore = false;
	chan_data->transfer_cfg.isSoftTriggerIgnore = false;
	SDMA_SubmitTransfer(&chan_data->handle, &chan_data->transfer_cfg);

	chan_data->state = SDMA_CHAN_STATE_CONFIGURED;
	sys_cache_data_flush_all();

	return 0;
}

static int dma_nxp_sdma_start(const struct device *dev, uint32_t channel)
{
	const struct sdma_dev_cfg *dev_cfg = dev->config;
	struct sdma_dev_data *dev_data = dev->data;
	struct sdma_channel_data *chan_data;

	if (channel >= FSL_FEATURE_SDMA_MODULE_CHANNEL) {
		LOG_ERR("%s: invalid channel %d", __func__, channel);
		return -EINVAL;
	}


	chan_data = &dev_data->chan[channel];

	SDMA_SetChannelPriority(dev_cfg->base, channel, DMA_NXP_SDMA_CHAN_DEFAULT_PRIO);
	SDMA_StartChannelSoftware(dev_cfg->base, channel);

	//sdma_dump_register_data(dev_cfg->base);

	LOG_INF("dma_nxp_sdma_start() chan %d", channel);
	return 0;
}

static int dma_nxp_sdma_stop(const struct device *dev, uint32_t channel)
{
	struct sdma_dev_data *dev_data = dev->data;
	struct sdma_channel_data *chan_data;

	if (channel >= FSL_FEATURE_SDMA_MODULE_CHANNEL) {
		LOG_ERR("%s: invalid channel %d", __func__, channel);
		return -EINVAL;
	}

	chan_data = &dev_data->chan[channel];

	SDMA_StopTransfer(&chan_data->handle);
	return 0;
}

static int dma_nxp_sdma_get_status(const struct device *dev, uint32_t channel,
				   struct dma_status *stat)
{
	struct sdma_dev_data *dev_data = dev->data;
	struct sdma_channel_data *chan_data;
	unsigned int key;

	chan_data = &dev_data->chan[channel];

	key = irq_lock();
	stat->free = chan_data->stat.free;
	stat->pending_length = chan_data->stat.pending_length;
	irq_unlock(key);

	return 0;
}

static int dma_nxp_sdma_reload(const struct device *dev, uint32_t channel, uint32_t src,
			       uint32_t dst, size_t size)
{
	struct sdma_dev_data *dev_data = dev->data;
	struct sdma_channel_data *chan_data;
	unsigned int key;

	chan_data = &dev_data->chan[channel];

	if (!size) {
		return 0;
	}

	key = irq_lock();
	if (chan_data->direction == MEMORY_TO_PERIPHERAL) {
		dma_nxp_sdma_produce(chan_data, size);
	} else {
		dma_nxp_sdma_consume(chan_data, size);
	}
	irq_unlock(key);

	return 0;
}

static int dma_nxp_sdma_get_attribute(const struct device *dev, uint32_t type, uint32_t *val)
{
	switch (type) {
	case DMA_ATTR_BUFFER_SIZE_ALIGNMENT:
		*val = 4;
		break;
	case DMA_ATTR_BUFFER_ADDRESS_ALIGNMENT:
		*val = 128; /* should be dcache_align */
		break;
	case DMA_ATTR_MAX_BLOCK_COUNT:
		*val = DMA_NXP_SDMA_BD_COUNT;
		break;
	default:
		LOG_ERR("invalid attribute type: %d", type);
		return -EINVAL;
	}
	return 0;
}

static bool sdma_channel_filter(const struct device *dev, int chan_id, void *param)
{
	struct sdma_dev_data *dev_data = dev->data;

	/* chan 0 is reserved for boot channel */
	if (chan_id == 0) {
		return false;
	}

	if (chan_id >= FSL_FEATURE_SDMA_MODULE_CHANNEL) {
		return false;
	}

	dev_data->chan[chan_id].event_source = *((int *)param);
	dev_data->chan[chan_id].index = chan_id;
	dev_data->chan[chan_id].capacity = 0;

	return true;
}

static DEVICE_API(dma, sdma_api) = {
	.reload = dma_nxp_sdma_reload,
	.config = dma_nxp_sdma_config,
	.start = dma_nxp_sdma_start,
	.stop = dma_nxp_sdma_stop,
	.suspend = dma_nxp_sdma_stop,
	.resume = dma_nxp_sdma_start,
	.get_status = dma_nxp_sdma_get_status,
	.get_attribute = dma_nxp_sdma_get_attribute,
	.chan_filter = sdma_channel_filter,
};

static int dma_nxp_sdma_init(const struct device *dev)
{
	struct sdma_dev_data *data = dev->data;
	const struct sdma_dev_cfg *cfg = dev->config;
	sdma_config_t defconfig;

	data->dma_ctx.magic = DMA_MAGIC;
	data->dma_ctx.dma_channels = FSL_FEATURE_SDMA_MODULE_CHANNEL;
	data->dma_ctx.atomic = data->channels_atomic;

	SDMA_GetDefaultConfig(&defconfig);
	defconfig.ratio = kSDMA_ARMClockFreq;

	SDMA_Init(cfg->base, &defconfig);

	/* configure interrupts */
	cfg->irq_config();

	return 0;
}

#define DMA_NXP_SDMA_INIT(inst)						\
	static ATOMIC_DEFINE(dma_nxp_sdma_channels_atomic_##inst,	\
			     FSL_FEATURE_SDMA_MODULE_CHANNEL);		\
	static struct __nocache sdma_dev_data sdma_data_##inst = {		\
		.channels_atomic = dma_nxp_sdma_channels_atomic_##inst,	\
	};								\
	static void dma_nxp_sdma_##inst_irq_config(void);		\
	static const struct sdma_dev_cfg sdma_cfg_##inst = {		\
		.base = (SDMAARM_Type *)DT_INST_REG_ADDR(inst),				\
		.irq_config = dma_nxp_sdma_##inst_irq_config,		\
	};								\
	static void dma_nxp_sdma_##inst_irq_config(void)		\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(inst),				\
			    DT_INST_IRQ(inst, priority),		\
			    dma_nxp_sdma_isr, DEVICE_DT_INST_GET(inst), 0);	\
		irq_enable(DT_INST_IRQN(inst));				\
	}								\
	DEVICE_DT_INST_DEFINE(inst, dma_nxp_sdma_init, NULL,		\
			      &sdma_data_##inst, &sdma_cfg_##inst,	\
			      PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,	\
			      &sdma_api);				\

DT_INST_FOREACH_STATUS_OKAY(DMA_NXP_SDMA_INIT);
