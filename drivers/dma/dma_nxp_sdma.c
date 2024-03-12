/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include "dma_nxp_sdma.h"

extern struct tr_ctx sdma_tr;

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

void dma_nxp_sdma_callback(sdma_handle_t *handle, void *userData, bool transferDone,
			    uint32_t bdIndex);

struct sdma_dev_cfg {
	SDMAARM_Type *base;
	void (*irq_config)(void);
};

struct sdma_channel_data {
	sdma_handle_t handle;
	sdma_buffer_descriptor_t *bd_list; /* pre-allocated list of BD used for transfer */
	sdma_buffer_descriptor_t *bd_current; /* curent BD used */
	uint32_t bd_count; /* number of BD in the bd_list */
	sdma_transfer_config_t transfer_cfg;
	sdma_peripheral_t peripheral;
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
}

void sdma_set_peripheral_type(struct dma_config *config, sdma_peripheral_t *type)
{
	*type = kSDMA_PeripheralNormal_SP;
}


/*static */int dma_nxp_sdma_channel_get(const struct device *dev, uint32_t channel)
{
	sdma_handle_t *handle;

	handle = DEV_SDMA_HANDLE(dev, channel);
	SDMA_CreateHandle(handle, DEV_BASE(dev), channel, NULL);

	return 0;
}

/*static*/ int dma_nxp_sdma_config(const struct device *dev, uint32_t channel,
		       struct dma_config *config)
{
	const struct sdma_dev_cfg *dev_cfg = dev->config;
	struct sdma_dev_data *dev_data = dev->data;
	struct sdma_channel_data *chan_data;
	struct dma_block_config *block_cfg;
	sdma_buffer_descriptor_t *iter_bd;

	/* TODO: verify channel index is valid */
	if (channel >= FSL_FEATURE_SDMA_MODULE_CHANNEL) {
		LOG_ERR("sdma_config() invalid channel %d", channel);
		return -EINVAL;
	}

	chan_data = &dev_data->chan[channel];
	chan_data->bd_list = &dev_data->bd_pool[channel][0];
	chan_data->bd_current = chan_data->bd_list;
	chan_data->bd_count = config->block_count;

	/* zero out the pre-allocated BD list */
	memset(chan_data->bd_list, 0, sizeof(sdma_buffer_descriptor_t) * chan_data->bd_count);

	SDMA_CreateHandle(&chan_data->handle, DEV_BASE(dev), channel, NULL);
	SDMA_InstallBDMemory(&chan_data->handle, chan_data->bd_list, chan_data->bd_count);

	sdma_set_transfer_type(config, &chan_data->transfer_cfg.type);
	sdma_set_peripheral_type(config, &chan_data->peripheral);

	block_cfg = config->head_block;
	iter_bd = chan_data->bd_list;

	for (int i = 0; i < config->block_count; i++) {
		bool isLast = false;
		bool isWrap = false;

		/* last block */
		if (i == config->block_count - 1) {
			isLast = true;
			isWrap = true;
		}

		SDMA_ConfigBufferDescriptor(iter_bd, block_cfg->source_address,
					    block_cfg->dest_address, config->source_data_size,
					    block_cfg->block_size, isLast, true, isWrap,
					    chan_data->transfer_cfg.type);

		block_cfg = block_cfg->next_block;
		iter_bd++;
	}

	/* prepare transfer with the first block */
	block_cfg = config->head_block;

	SDMA_PrepareTransfer(&chan_data->transfer_cfg, block_cfg->source_address,
			     block_cfg->dest_address, config->source_data_size,
			     config->dest_data_size,
			     block_cfg->block_size, block_cfg->block_size,
			     0, chan_data->peripheral, chan_data->transfer_cfg.type);
	
	SDMA_SubmitTransfer(&chan_data->handle, &chan_data->transfer_cfg);

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
	SDMA_StartTransfer(DEV_SDMA_HANDLE(dev, channel));
	return 0;
}


static int sdma_reload(const struct device *dev, uint32_t channel, uint32_t src,
		       uint32_t dst, size_t size)
{
	SubmitTransfer(
#if 0
	struct sdma_channel *sdma_chan;

	sdma_chan = DEV_CHANNEL_DATA(dev, channel);

	SDMA_PrepareTransfer(&sdma_chan->transfer_config,
			     (void *)src,
			     (void *)dst,
			     size, /*FIXME*/
			     size, /*FIXME */
			     size, /*FIXME*/
			     size,
			     0, //eventSource
			     0,//sdma_chan->peripheral_type,
			     0);//sdma_chan->transfer_type);
#endif

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
	.reload = sdma_reload,
	.config = dma_nxp_sdma_config,
	.start = dma_nxp_sdma_start,
	.stop = dma_nxp_sdma_stop,
	.suspend = dma_nxp_sdma_stop,
	.resume = dma_nxp_sdma_start,
	.get_status = dma_nxp_sdma_get_status,
	.get_attribute = dma_nxp_sdma_get_attribute,
//	.chan_filter = sdma_channel_filter,
};

/*static*/ int dma_nxp_sdma_init(const struct device *dev)
{
	const struct sdma_dev_cfg *cfg = dev->config;
	sdma_config_t defconfig;

	SDMA_GetDefaultConfig(&defconfig);
	SDMA_Init(cfg->base, &defconfig);

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
