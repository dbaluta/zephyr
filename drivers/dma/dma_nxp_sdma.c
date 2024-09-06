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
#define DEV_CHANNEL_DATA(dev, chan) ((struct sdma_channel_data *)(&DEV_DATA(dev)->channel_data[chan]))
#define DEV_SDMA_HANDLE(dev, chan) ((sdma_handle_t *)(&(DEV_CHANNEL_DATA(dev, chan)->handle)))

#define SDMA_MAX_BD	2

K_HEAP_DEFINE(sdma_buffer_pool,
	      FSL_FEATURE_SDMA_MODULE_CHANNEL * 
	      SDMA_MAX_BD *
	      sizeof(sdma_buffer_descriptor_t));

void do_log(char const *p) 
{
}

void dma_nxp_sdma_callback(sdma_handle_t *handle, void *userData, bool transferDone,
			    uint32_t bdIndex);

struct sdma_dev_cfg {
	SDMAARM_Type *base;
};

struct sdma_channel_data {
	sdma_handle_t handle;
	sdma_transfer_config_t transferConfig;
	uint8_t channel;

};

struct sdma_dev_data {
	struct sdma_channel_data channel_data[32];
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
void sdma_set_transfer_type(const struct device *dev, uint32_t channel,
			    struct dma_config *config)
{
#if 0
	sdma_transfer_type_t transfer_type;
	struct sdma_channel *sdma_chan;

	sdma_chan = DEV_CHANNEL_DATA(dev, channel);

	switch (config->channel_direction) {
		case MEMORY_TO_MEMORY:
			transfer_type = kSDMA_MemoryToMemory;
			break;
		case MEMORY_TO_PERIPHERAL:
			transfer_type = kSDMA_MemoryToPeripheral;
			break;
		case PERIPHERAL_TO_MEMORY:
			transfer_type = kSDMA_PeripheralToMemory;
			break;
		case PERIPHERAL_TO_PERIPHERAL:
			transfer_type = kSDMA_PeripheralToPeripheral;
			break;
		default:
			LOG_ERR("channel direction not supported%d\n", config->channel_direction);
			return;
	}

	sdma_chan->transfer_config.type = transfer_type;
#endif
	return;
}

void sdma_set_peripheral_type(const struct device *dev, uint32_t channel,
			    struct dma_config *config)
{
#if 0
	struct sdma_channel *sdma_chan;

	sdma_chan = DEV_CHANNEL_DATA(dev, channel);

	sdma_chan->peripheral_type = kSDMA_PeripheralNormal;
#endif
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
	struct sdma_dev_data *data;
	struct sdma_dev_cfg * config;

#if 0
/*static*/ int dma_nxp_sdma_config(const struct device *dev, uint32_t channel,
		       struct dma_config *config)
{
	int i;
	sdma_handle_t *handle;
	struct sdma_data *dma_data;
	const struct sdma_config *dev_config;
	struct sdma_channel *sdma_chan;
	//sdma_peripheral_t peripheral_type;
	struct dma_block_config *block_config;

	LOG_ERR("dma_nxp_sdma_config starting");

	LOG_INF("starting sdma_config chan %d\n", channel);

	dma_data = dev->data;
	dev_config = dev->config;

	if (channel >= FSL_FEATURE_SDMA_MODULE_CHANNEL) {
		LOG_ERR("sdma_config() invalid channel %d", channel);
		return -EINVAL;
	}

//	do_log();
	LOG_INF("get handle");
	handle = DEV_SDMA_HANDLE(dev, channel);
	sdma_chan = DEV_CHANNEL_DATA(dev, channel);


	LOG_INF("get handle %x", (int)handle);


//	do_log();

	if (sdma_chan->id == -1) {
		LOG_INF("channel id is -1 calling createhandle %d", channel);
		sdma_chan->id = channel;
		SDMA_CreateHandle(handle, DEV_BASE(dev), channel, NULL);

		sdma_chan->BDCount = 2;
		sdma_chan->BDPool = k_heap_alloc(&sdma_buffer_pool,
						 sdma_chan->BDCount * 
						 sizeof(*sdma_chan->BDPool),
						 K_FOREVER);
		if (sdma_chan == NULL)
			return -ENOMEM;

		SDMA_InstallBDMemory(handle, sdma_chan->BDPool, sdma_chan->BDCount);
		SDMA_SetCallback(handle, dma_nxp_sdma_callback, sdma_chan);
	}


	sdma_set_transfer_type(dev, channel, config);
	sdma_set_peripheral_type(dev, channel, config);


	block_config = config->head_block;

	for (i = 0; i < sdma_chan->BDCount; i++) {
		bool isLast = false;
		bool isWrap = false;

		if (i == sdma_chan->BDCount - 1) {
			isLast = true;
			isWrap = true;
		}

		SDMA_ConfigBufferDescriptor(&sdma_chan->BDPool[i],
					    block_config->source_address,
					    block_config->dest_address,
					    config->source_data_size,
					    block_config->block_size,
					    isLast,
					    true,
					    isWrap,
					    sdma_chan->transfer_config.type);

		block_config = block_config->next_block;

	}
	SDMA_PrepareTransfer(&sdma_chan->transfer_config,
			     block_config->source_address,
			     block_config->dest_address,
			     config->source_data_size,
			     config->dest_data_size,
			     config->source_burst_length,
			     block_config->block_size,
			     0, //eventSource
			     sdma_chan->peripheral_type,
			     sdma_chan->transfer_config.type);

	SDMA_SubmitTransfer(handle, &sdma_chan->transfer_config);
#endif
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
			*val = 2;
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
	struct sdma_dev_cfg *cfg = dev->cfg;
	sdma_config_t defconfig

	
	SDMA_GetDefaultconfig(&defconfig);
	SDMA_Init(cfg->base, defconfig);

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
