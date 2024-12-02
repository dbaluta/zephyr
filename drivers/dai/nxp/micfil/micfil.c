/*
 * Copyright 2024 NXP
 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/dai.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#include "fsl_pdm.h"

void do_log(char *str)
{
//	LOG_INF("ana");
}

#define DT_DRV_COMPAT nxp_dai_micfil

LOG_MODULE_REGISTER(nxp_dai_micfil);

#define MICFIL_DMA_HANDSHAKE(inst, dir) \
	((DT_INST_DMAS_CELL_BY_NAME(inst, dir, channel) & GENMASK(7,0)) | \
	((DT_INST_DMAS_CELL_BY_NAME(inst, dir, mux) << 8) & GENMASK(15, 8)))

struct dai_nxp_micfil_data {
	int quality;
	struct dai_config cfg;
};

struct dai_nxp_micfil_config {
	PDM_Type *base;
	pdm_config_t config;
	pdm_channel_config_t chan_config;
	const struct dai_properties *rx_props;
	int irq;
	void (*irq_config)(void);
};

/* MICFIL configuration - SOF_DAI_MICFIL_CONFIG */
struct micfil_bespoke_config {
	uint32_t pdm_rate;
	uint32_t pdm_ch;
};

void pdm_print_regs(PDM_Type *base) {
	LOG_INF("CTRL_1 %08x", base->CTRL_1);
	LOG_INF("CTRL_2 %08x", base->CTRL_2);
	LOG_INF("STAT %08x", base->STAT);
	LOG_INF("FIFO_CTRL %08x", base->FIFO_CTRL);
	LOG_INF("FIFO_STAT %08x", base->FIFO_STAT);
	LOG_INF("DATA_CH0 %08x", base->DATACH[0]);
	LOG_INF("DC_CTRL %08x", base->DC_CTRL);
	LOG_INF("RANGE_CTRL %08x", base->RANGE_CTRL);
	LOG_INF("RANGE_STAT %08x", base->RANGE_STAT);
}

#if 0
/* On DMIC IRQ event trace the status register that contains the status and
 * error bit fields.
 */
static void dai_nxp_micfil_irq_handler(const void *data)
{
}

#endif
static void dai_nxp_micfil_trigger_start(const struct device *dev)
{
	struct dai_nxp_micfil_data *micfil = dev->data;
	struct dai_nxp_micfil_config *dev_cfg = dev->config;

	LOG_INF("dmic_nxp_micfil_start()");

	//PDM_Reset(dev_cfg->base);

	/* enable DMA requests */
	PDM_EnableDMA(dev_cfg->base, true);

	/* enable the module */
	PDM_Enable(dev_cfg->base, true);

	pdm_print_regs(dev_cfg->base);
}

static void dai_nxp_micfil_trigger_stop(const struct device *dev)
{
	struct dai_nxp_micfil_data *micfil = dev->data;
	struct dai_nxp_micfil_config *cfg = dev->config;
	LOG_INF("dmic_nxp_micfil_trigger_stop()");

	/* disable DMA requests */
	PDM_EnableDMA(cfg->base, false);

	/* disable module */
	PDM_Enable(cfg->base, false);

}

const struct dai_properties
*dai_nxp_micfil_get_properties(const struct device *dev,
			       enum dai_dir dir,
			       int stream_id)
{
	struct dai_nxp_micfil_config *cfg = dev->config;

	LOG_INF("get_properties ... for dir = %d", dir);

	if (dir == DAI_DIR_RX)
		return cfg->rx_props;
	else {
		LOG_ERR("micfil: invalid direction %d", dir);
		return NULL;
	}
}

static int dai_nxp_micfil_trigger(const struct device *dev, enum dai_dir dir,
				  enum dai_trigger_cmd cmd)
{
	LOG_DBG("dmic_trigger()");

	if (dir != DAI_DIR_RX) {
		LOG_ERR("dai_nxp_micfil_trigger(): direction != DAI_DIR_RX");
		return -EINVAL;
	}

	switch (cmd) {
	case DAI_TRIGGER_START:
	dai_nxp_micfil_trigger_start(dev);
		break;
	case DAI_TRIGGER_STOP:
	case DAI_TRIGGER_PAUSE:
		dai_nxp_micfil_trigger_stop(dev);
		break;
	/* nothing to do here, return 0 */
	case DAI_TRIGGER_PRE_START:
	case DAI_TRIGGER_COPY:
		return 0;
	default:
		LOG_ERR("dai_nxp_micfil_trigger(), invalid trigger cmd %d", cmd);
		return -EINVAL;
	}

	return 0;
}

static int dai_nxp_micfil_get_config(const struct device *dev, struct dai_config *cfg, enum dai_dir dir)
{
	struct dai_nxp_micfil_data *micfil = dev->data;

	LOG_INF("... get_config()");

	memcpy(cfg, &micfil->cfg, sizeof(*cfg));
	LOG_INF("get_config() ch %d rate %d", cfg->channels, cfg->rate);
	return 0;
}

static int dai_nxp_micfil_set_config(const struct device *dev,
		const struct dai_config *cfg, const void *bespoke_cfg)

{
	struct dai_nxp_micfil_data *micfil = dev->data;
	struct dai_nxp_micfil_config *dev_cfg = dev->config;
	struct micfil_bespoke_config *micfil_cfg = bespoke_cfg;
	int i;

	LOG_INF("set_config ... %x", (int)micfil_cfg);
	if (micfil_cfg) 
		LOG_INF("set_config ch %d rate %d", micfil_cfg->pdm_rate, micfil_cfg->pdm_ch);

	dev_cfg->config.qualityMode = kPDM_QualityModeVeryLow0;
	dev_cfg->config.fifoWatermark = 31;
	dev_cfg->config.cicOverSampleRate = 16;

	LOG_INF("PDM INit..");
	PDM_Init(dev_cfg->base, &dev_cfg->config);

	for (i = 0; i < micfil_cfg->pdm_ch; i++) {
		dev_cfg->chan_config.gain = kPDM_DfOutputGain2;
		dev_cfg->chan_config.cutOffFreq = kPDM_DcRemoverBypass;
		//dev_cfg->chan_config.outputCutOffFreq = kPDM_DcRemoverBypass;
		PDM_SetChannelConfig(dev_cfg->base, i, &dev_cfg->chan_config);
	}

	PDM_SetSampleRateConfig(dev_cfg->base, 24576000, 48000);

	pdm_print_regs(dev_cfg->base);

	return 0;
}

static int dai_nxp_micfil_probe(const struct device *dev)
{

	LOG_INF("micfil_probe()");

	return 0;
}


static int dai_nxp_micfil_remove(const struct device *dev)
{
	return 0;
}

const struct dai_driver_api dai_nxp_micfil_ops = {
	.probe			= dai_nxp_micfil_probe,
	.remove			= dai_nxp_micfil_remove,
	.config_set		= dai_nxp_micfil_set_config,
	.config_get		= dai_nxp_micfil_get_config,
	.get_properties		= dai_nxp_micfil_get_properties,
	.trigger		= dai_nxp_micfil_trigger,
};

static int micfil_init(const struct device *dev)
{
	struct dai_nxp_micfil_data *micfil = dev->data;
	struct dai_nxp_micfil_config *dev_cfg = dev->config;

	LOG_INF("micfil_init(base) %x", dev_cfg->base);
	LOG_INF("micfil dai_index %d type %d", micfil->cfg.dai_index, micfil->cfg.type);

	return 0;
}

#define DAI_NXP_MICFIL_INIT(inst)							\
	static const struct dai_properties micfil_rx_props_##inst = {	\
		.fifo_address = (0x30ca0000 + 0x24),	\
		.fifo_depth = 32,	\
		.dma_hs_id = MICFIL_DMA_HANDSHAKE(inst, rx),	\
	}; \
	/* static void dai_nxp_micfil_##inst_irq_config(void);	*/			\
	static const struct dai_nxp_micfil_config dai_nxp_micfil_config_##inst = {		\
		.base = (PDM_Type *)DT_INST_REG_ADDR(inst),				\
		.rx_props = &micfil_rx_props_##inst,	\
		/*.irq_config = dai_nxp_micfil_##inst_irq_config,	*/			\
	};										\
	static struct dai_nxp_micfil_data dai_nxp_micfil_data_##inst = {				\
		.cfg.type = DAI_IMX_MICFIL,					\
		.cfg.dai_index = 2,				\
	};				\
	/*void irq_config_##inst(void) 							\
	{										\
		IRQ_CONNECT(DT_INST_IRQN(inst), 					\
			DT_INST_IRQ_(inst, priority),					\
			NULL,						\
			DEVICE_DT_INST_GET(inst), 0);					\
	}	*/									\
	DEVICE_DT_INST_DEFINE(inst, &micfil_init, NULL,				\
		      &dai_nxp_micfil_data_##inst, &dai_nxp_micfil_config_##inst,	\
		      POST_KERNEL, CONFIG_DAI_INIT_PRIORITY,				\
		      &dai_nxp_micfil_ops);						\

DT_INST_FOREACH_STATUS_OKAY(DAI_NXP_MICFIL_INIT)
