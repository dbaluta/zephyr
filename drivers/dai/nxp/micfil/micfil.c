/*
 * Copyright 2024 NXP
 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/dai.h>
#include <zephyr/drivers/device.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT dai_nxp_micfil

struct micfil_data {
};

struct micfil_config {
};


/* On DMIC IRQ event trace the status register that contains the status and
 * error bit fields.
 */
static void dai_nxp_micfil_irq_handler(const void *data)
{
}

static int dai_nxp_micfil_probe(struct dai_intel_dmic *dmic)
{
	LOG_INF("dai_nxp_micfil_probe()");

	return 0;
}

static int dai_nxp_micfil_remove(struct dai_intel_dmic *dmic)
{
	LOG_INF("dai_nxp_micfil_remove()");

	return 0;
}

static void dai_nxp_micfil_start(struct dai_intel_dmic *dmic)
{

	LOG_INF("dmic_nxp_micfil_start()");
}

static void dai_nxp_micfil_stop(struct dai_intel_dmic *dmic, bool stop_is_pause)
{
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
	struct dai_intel_dmic *dmic = (struct dai_intel_dmic *)dev->data;

	LOG_DBG("dmic_trigger()");

	if (dir != DAI_DIR_RX) {
		LOG_ERR("dmic_trigger(): direction != DAI_DIR_RX");
		return -EINVAL;
	}

	switch (cmd) {
	case DAI_TRIGGER_START:
		if (dmic->state == DAI_STATE_PAUSED ||
		    dmic->state == DAI_STATE_PRE_RUNNING) {
			dai_nxp_micfil_start(dmic);
			dmic->state = DAI_STATE_RUNNING;
		} else {
			LOG_ERR("dmic_trigger(): state is not prepare or paused, dmic->state = %u",
				dmic->state);
		}
		break;
	case DAI_TRIGGER_STOP:
		dai_nxp_micfil_stop(dmic, false);
		dmic->state = DAI_STATE_PRE_RUNNING;
		break;
	case DAI_TRIGGER_PAUSE:
		dai_nxp_micfil_stop(dmic, true);
		dmic->state = DAI_STATE_PAUSED;
		break;
	case DAI_TRIGGER_COPY:
		dai_nxp_micfil_gain_ramp(dmic);
		break;
	default:
		break;
	}

	return 0;
}

static int dai_nxp_micfil_get_config(const struct device *dev, struct dai_config *cfg, enum dai_dir dir)
{
}

static int dai_nxp_micfil_set_config(const struct device *dev,
		const struct dai_config *cfg, const void *bespoke_cfg)

{
}

static int dai_nxp_micfil_probe(const struct device *dev)
{
}

static int dai_nxp_micfil_remove(const struct device *dev)
{
	struct dai_intel_dmic *dmic = (struct dai_intel_dmic *)dev->data;

	return ret;
}

const struct dai_driver_api dai_nxp_micfil_ops = {
	.probe			= dai_nxp_micfil_probe,
	.remove			= dai_nxp_micfil_remove,
	.config_set		= dai_nxp_micfil_set_config,
	.config_get		= dai_nxp_micfil_get_config,
	.get_properties		= dai_nxp_micfil_get_properties,
	.trigger		= dai_nxp_micfil_trigger,
};

#define DAI_NXP_MICFIL_INIT(inst)							\
void irq_config_##inst(void) 								\
{											\
	IRQ_CONNECT(DT_INST_IRQN(inst), 						\
		    0,									\
		    micfil_isr								\
		    DEVICE_DT_INST_GET(inst),						\
		    0);									\
}											\
static struct micfil_config micfil_config_##inst = {					\
};											\
DEVICE_DT_INST_DEFINE(inst, &dai_nxp_micfil_init, PM_DEVICE_DT_INST_GET(inst),		\
		      &dai_nxp_micfil_data_##inst, &dai_nxp_micfil_config_##inst,	\
		      POST_KERNEL, CONFIG_DAI_INIT_PRIORITY,				\
		      &dai_nxp_micfil_ops);						\

DT_INST_FOREACH_STATUS_OKAY(DAI_NXP_MICFIL_INIT)
