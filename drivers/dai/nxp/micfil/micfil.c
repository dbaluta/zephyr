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

#define DT_DRV_COMPAT nxp_dai_micfil

LOG_MODULE_REGISTER(nxp_dai_micfil);

struct dai_nxp_micfil_data {
	int quality;
	struct dai_config cfg;
};

struct dai_nxp_micfil_config {
	PDM_Type *base;
	pdm_config_t config;
	int irq;
	void (*irq_config)(void);
};

/* MICFIL configuration - SOF_DAI_MICFIL_CONFIG */
struct micfil_bespoke_config {
	uint32_t pdm_rate;
	uint32_t pdm_ch;
};

/* On DMIC IRQ event trace the status register that contains the status and
 * error bit fields.
 */
static void dai_nxp_micfil_irq_handler(const void *data)
{
}

static void dai_nxp_micfil_trigger_start(const struct device *dev)
{
	struct dai_nxp_micfil_data *micfil = dev->data;
	struct dai_nxp_micfil_config *dev_cfg = dev->config;

	LOG_INF("dmic_nxp_micfil_start()");

	PDM_Reset(dev_cfg->base);

	/* enable DMA requests */
	PDM_EnableDMA(dev_cfg->base, true);

	/* enable the module */
	PDM_Enable(dev_cfg->base, true);
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
	case DAI_TRIGGER_COPY:
		break;
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

	return 0;
}

static int dai_nxp_micfil_set_config(const struct device *dev,
		const struct dai_config *cfg, const void *bespoke_cfg)

{
	struct dai_nxp_micfil_data *micfil = dev->data;
	struct dai_nxp_micfil_config *dev_cfg = dev->config;
	struct micfil_bespoke_config *micfil_cfg = bespoke_cfg;
	int i;

	LOG_INF("set_config ch %d rate %d", micfil_cfg->pdm_rate, micfil_cfg->pdm_ch);

	PDM_Init(dev_cfg->base, &dev_cfg->config);

	for (i = 0; i < micfil_cfg->pdm_ch; i++)
		PDM_SetChannelConfig(dev_cfg->base, i, &dev_cfg->config);
	PDM_SetSampleRateConfig(dev_cfg->base, 0, 48000);

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
	/* static void dai_nxp_micfil_##inst_irq_config(void);	*/			\
	static const struct dai_nxp_micfil_config dai_nxp_micfil_config_##inst = {		\
		.base = (PDM_Type *)DT_INST_REG_ADDR(inst),				\
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
