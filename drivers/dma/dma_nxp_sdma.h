/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_DMA_DMA_NXP_SDMA_H_
#define ZEPHYR_DRIVERS_DMA_DMA_NXP_SDMA_H_

#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>

#include "fsl_sdma.h"


/*static*/ int dma_nxp_sdma_start(const struct device *dev, uint32_t channel);
/*static*/ int dma_nxp_sdma_stop(const struct device *dev, uint32_t channel);

LOG_MODULE_REGISTER(nxp_sdma);

/* used for driver binding */
#define DT_DRV_COMPAT nxp_sdma

/* workaround the fact that device_map() is not defined for SoCs with no MMU */
#ifndef DEVICE_MMIO_IS_IN_RAM
#define device_map(virt, phys, size, flags) *(virt) = (phys)
#endif /* DEVICE_MMIO_IS_IN_RAM */

/* macros used to parse DTS properties */

/* used in conjunction with LISTIFY which expects F to also take a variable
 * number of arguments. Since IDENTITY doesn't do that we need to use a version
 * of it which also takes a variable number of arguments.
 */
#define IDENTITY_VARGS(V, ...) IDENTITY(V)

/* used to generate an array of indexes for the channels */
#define _EDMA_CHANNEL_INDEX_ARRAY(inst)\
	LISTIFY(DT_INST_PROP_LEN_OR(inst, valid_channels, 0), IDENTITY_VARGS, (,))

/* used to generate an array of indexes for the channels - this is different
 * from _EDMA_CHANNEL_INDEX_ARRAY because the number of channels is passed
 * explicitly through dma-channels so no need to deduce it from the length
 * of the valid-channels property.
 */
#define _EDMA_CHANNEL_INDEX_ARRAY_EXPLICIT(inst)\
	LISTIFY(DT_INST_PROP_OR(inst, dma_channels, 0), IDENTITY_VARGS, (,))

/* used to generate an array of indexes for the interrupt */
#define _EDMA_INT_INDEX_ARRAY(inst)\
	LISTIFY(DT_NUM_IRQS(DT_INST(inst, DT_DRV_COMPAT)), IDENTITY_VARGS, (,))

/* used to register an ISR/arg pair. TODO: should we also use the priority? */
#define _EDMA_INT_CONNECT(idx, inst)				\
	IRQ_CONNECT(DT_INST_IRQN_BY_IDX(inst, idx),		\
		    0, edma_isr,				\
		    &channels_##inst[idx], 0)

/* used to declare a struct edma_channel by the non-explicit macro suite */
#define _EDMA_CHANNEL_DECLARE(idx, inst)				\
{									\
	.id = DT_INST_PROP_BY_IDX(inst, valid_channels, idx),		\
	.dev = DEVICE_DT_INST_GET(inst),				\
	.irq = DT_INST_IRQN_BY_IDX(inst, idx),				\
}

/* used to declare a struct edma_channel by the explicit macro suite */
#define _EDMA_CHANNEL_DECLARE_EXPLICIT(idx, inst)			\
{									\
	.id = idx,							\
	.dev = DEVICE_DT_INST_GET(inst),				\
	.irq = DT_INST_IRQN_BY_IDX(inst, idx),				\
}

/* used to create an array of channel IDs via the valid-channels property */
#define _EDMA_CHANNEL_ARRAY(inst)					\
	{ FOR_EACH_FIXED_ARG(_EDMA_CHANNEL_DECLARE, (,),		\
			     inst, _EDMA_CHANNEL_INDEX_ARRAY(inst)) }

/* used to create an array of channel IDs via the dma-channels property */
#define _EDMA_CHANNEL_ARRAY_EXPLICIT(inst)				\
	{ FOR_EACH_FIXED_ARG(_EDMA_CHANNEL_DECLARE_EXPLICIT, (,), inst,	\
			     _EDMA_CHANNEL_INDEX_ARRAY_EXPLICIT(inst)) }

/* used to construct the channel array based on the specified property:
 * dma-channels or valid-channels.
 */
#define EDMA_CHANNEL_ARRAY_GET(inst)							\
	COND_CODE_1(DT_NODE_HAS_PROP(DT_INST(inst, DT_DRV_COMPAT), dma_channels),	\
		    (_EDMA_CHANNEL_ARRAY_EXPLICIT(inst)),				\
		    (_EDMA_CHANNEL_ARRAY(inst)))

#define EDMA_HAL_CFG_GET(inst)								\
	COND_CODE_1(DT_NODE_HAS_PROP(DT_INST(inst, DT_DRV_COMPAT), hal_cfg_index),	\
		    (s_edmaConfigs[DT_INST_PROP(inst, hal_cfg_index)]),			\
		    (s_edmaConfigs[0]))

/* used to register edma_isr for all specified interrupts */
#define EDMA_CONNECT_INTERRUPTS(inst)				\
	FOR_EACH_FIXED_ARG(_EDMA_INT_CONNECT, (;),		\
			   inst, _EDMA_INT_INDEX_ARRAY(inst))

#define EDMA_CHANS_ARE_CONTIGUOUS(inst)\
	DT_NODE_HAS_PROP(DT_INST(inst, DT_DRV_COMPAT), dma_channels)

/* utility macros */

/* a few words about EDMA_CHAN_PRODUCE_CONSUME_{A/B}:
 *	- in the context of cyclic buffers we introduce
 *	the concepts of consumer and producer channels.
 *
 *	- a consumer channel is a channel for which the
 *	DMA copies data from a buffer, thus leading to
 *	less data in said buffer (data is consumed with
 *	each transfer).
 *
 *	- a producer channel is a channel for which the
 *	DMA copies data into a buffer, thus leading to
 *	more data in said buffer (data is produced with
 *	each transfer).
 *
 *	- for consumer channels, each DMA interrupt will
 *	signal that an amount of data has been consumed
 *	from the buffer (half of the buffer size if
 *	HALFMAJOR is enabled, the whole buffer otherwise).
 *
 *	- for producer channels, each DMA interrupt will
 *	signal that an amount of data has been added
 *	to the buffer.
 *
 *	- to signal this, the ISR uses EDMA_CHAN_PRODUCE_CONSUME_A
 *	which will "consume" data from the buffer for
 *	consumer channels and "produce" data for
 *	producer channels.
 *
 *	- since the upper layers using this driver need
 *	to let the EDMA driver know whenever they've produced
 *	(in the case of consumer channels) or consumed
 *	data (in the case of producer channels) they can
 *	do so through the reload() function.
 *
 *	- reload() uses EDMA_CHAN_PRODUCE_CONSUME_B which
 *	for consumer channels will "produce" data and
 *	"consume" data for producer channels, thus letting
 *	the driver know what action the upper layer has
 *	performed (if the channel is a consumer it's only
 *	natural that the upper layer will write/produce more
 *	data to the buffer. The same rationale applies to
 *	producer channels).
 *
 *	- EDMA_CHAN_PRODUCE_CONSUME_B is just the opposite
 *	of EDMA_CHAN_PRODUCE_CONSUME_A. If one produces
 *	data, the other will consume and vice-versa.
 *
 *	- all of this information is valid only in the
 *	context of cyclic buffers. If this behaviour is
 *	not enabled, querying the status will simply
 *	resolve to querying CITER and BITER.
 */
#define EDMA_CHAN_PRODUCE_CONSUME_A(chan, size)\
	((chan)->type == CHAN_TYPE_CONSUMER ?\
	 edma_chan_cyclic_consume(chan, size) :\
	 edma_chan_cyclic_produce(chan, size))

#define EDMA_CHAN_PRODUCE_CONSUME_B(chan, size)\
	((chan)->type == CHAN_TYPE_CONSUMER ?\
	 edma_chan_cyclic_produce(chan, size) :\
	 edma_chan_cyclic_consume(chan, size))

enum channel_type {
	CHAN_TYPE_CONSUMER = 0,
	CHAN_TYPE_PRODUCER,
};

enum channel_state {
	CHAN_STATE_INIT = 0,
	CHAN_STATE_CONFIGURED,
	CHAN_STATE_STARTED,
	CHAN_STATE_STOPPED,
	CHAN_STATE_SUSPENDED,
};

static inline int get_transfer_type(enum dma_channel_direction dir, uint32_t *type)
{
	switch (dir) {
	case MEMORY_TO_MEMORY:
		*type = kSDMA_MemoryToMemory;
		break;
	case MEMORY_TO_PERIPHERAL:
		*type = kSDMA_MemoryToPeripheral;
		break;
	case PERIPHERAL_TO_MEMORY:
		*type = kSDMA_PeripheralToMemory;
		break;
	default:
		LOG_ERR("invalid channel direction: %d", dir);
		return -EINVAL;
	}

	return 0;
}

static inline bool data_size_is_valid(uint16_t size)
{
	switch (size) {
	case 1:
	case 2:
	case 4:
	case 8:
	case 16:
	case 32:
	case 64:
		break;
	default:
		return false;
	}

	return true;
}

/* TODO: we may require setting the channel type through DTS
 * or through struct dma_config. For now, we'll only support
 * MEMORY_TO_PERIPHERAL and PERIPHERAL_TO_MEMORY directions
 * and assume that these are bound to a certain channel type.
 */
#if 0
static inline int edma_set_channel_type(struct sdma_channel *chan,
					enum dma_channel_direction dir)
{
	switch (dir) {
	case MEMORY_TO_PERIPHERAL:
		chan->type = CHAN_TYPE_CONSUMER;
		break;
	case PERIPHERAL_TO_MEMORY:
		chan->type = CHAN_TYPE_PRODUCER;
		break;
	default:
		LOG_ERR("unsupported transfer direction: %d", dir);
		return -ENOTSUP;
	}

	return 0;
}
#endif
#endif /* ZEPHYR_DRIVERS_DMA_DMA_NXP_SDMA_H_ */
