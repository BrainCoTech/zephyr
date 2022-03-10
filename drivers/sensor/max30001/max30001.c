/*
 * Copyright (c) 2022 BrainCo Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT maxim_max30001

#include <errno.h>
#include <kernel.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <drivers/sensor.h>
#include <soc.h>

#include "max30001.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(max30001, CONFIG_SENSOR_LOG_LEVEL);

struct max30001_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec intb;
	uint32_t ecg_sample_rate;
#ifdef CONFIG_MAX30001_LEAD_DETECTION
	uint32_t lead_off_imag;
	uint32_t lead_off_vth;
#endif
};
struct max30001_data {
	const struct device *dev;
	struct gpio_callback intb_cb;
	int ecg_sample;
	sensor_trigger_handler_t ecg_sample_handler;
#ifdef CONFIG_MAX30001_LEAD_DETECTION
	bool lead_status;
	sensor_trigger_handler_t lead_status_handler;
#endif
#if defined(CONFIG_MAX30001_TRIGGER_OWN_THREAD)
	struct k_sem sem;
	struct k_thread thread;

	K_KERNEL_STACK_MEMBER(stack,
			CONFIG_MAX30001_THREAD_STACK_SIZE);
#elif defined(CONFIG_MAX30001_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
};

static int max30001_read_reg(const struct device *dev, uint8_t reg,
			     uint32_t *val)
{
	const struct max30001_config *cfg = dev->config;
	uint8_t buf[4];
	struct spi_buf tx_buf[2];
	struct spi_buf rx_buf[2];
	struct spi_buf_set tx;
	struct spi_buf_set rx;
	int ret;

	/* Register address + R */
	buf[0] = (reg << 1) | BIT(0);

	tx_buf[0].buf = &buf[0];
	tx_buf[0].len = 1U;
	tx_buf[1].buf = NULL;
	tx_buf[1].len = 3U;

	rx_buf[0].buf = NULL;
	rx_buf[0].len = 1U;
	rx_buf[1].buf = &buf[1];
	rx_buf[1].len = 3U;

	tx.buffers = tx_buf;
	tx.count = 2U;

	rx.buffers = rx_buf;
	rx.count = 2U;

	ret = spi_transceive_dt(&cfg->bus, &tx, &rx);
	if (ret < 0) {
		return ret;
	}

	/* Received bytes is MSB first. */
	*val  = (uint32_t)buf[1] << 16;
	*val |= (uint32_t)buf[2] << 8;
	*val |= (uint32_t)buf[3];

	return 0;
}

static int max30001_write_reg(const struct device *dev, uint8_t reg,
			      uint32_t val)
{
	const struct max30001_config *cfg = dev->config;
	uint8_t buf[4];
	struct spi_buf tx_buf;
	struct spi_buf_set tx;

	/* Register address + W */
	buf[0] = reg << 1;

	/* Transmitted bytes should be MSB first. */
	buf[1] = (uint8_t)(val >> 16);
	buf[2] = (uint8_t)(val >> 8);
	buf[3] = (uint8_t)val;

	tx_buf.buf = buf;
	tx_buf.len = 4U;

	tx.buffers = &tx_buf;
	tx.count = 1U;

	return spi_write_dt(&cfg->bus, &tx);
}

static void max30001_intb_callback(const struct device *port,
				   struct gpio_callback *cb,
				   uint32_t pins)
{
	struct max30001_data *data =
		CONTAINER_OF(cb, struct max30001_data, intb_cb);

#if defined(CONFIG_MAX30001_TRIGGER_OWN_THREAD)
	k_sem_give(&data->sem);
#elif defined(CONFIG_MAX30001_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->work);
#endif
}

static void max30001_thread_cb(const struct device *dev)
{
	struct max30001_data *data = dev->data;
	struct sensor_trigger ecg_trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_VOLTAGE
	};
#ifdef CONFIG_MAX30001_LEAD_DETECTION
	struct sensor_trigger lead_trig = {
		.type = SENSOR_TRIG_NEAR_FAR,
		.chan = SENSOR_CHAN_PROX
	};
#endif
	uint32_t stat;

	if (max30001_read_reg(dev, MAX30001_STATUS, &stat) < 0) {
		LOG_ERR("Failed to read data");
		return;
	}

#ifdef CONFIG_MAX30001_LEAD_DETECTION
	/* Lead-on event */
	if ((stat & STATUS_LONINT) && data->lead_status_handler) {
		data->lead_status = true;
		data->lead_status_handler(dev, &lead_trig);
	}

	/* Lead-off event */
	if ((stat & STATUS_DCLOFFINT) && data->lead_status_handler) {
		data->lead_status = false;
		data->lead_status_handler(dev, &lead_trig);
	}
#endif /* CONFIG_MAX30001_LEAD_DETECTION */

	/* ECG sampling data ready */
	if ((stat & STATUS_EINT) && data->ecg_sample_handler) {
		data->ecg_sample_handler(dev, &ecg_trig);
	}
}

#ifdef CONFIG_MAX30001_TRIGGER_OWN_THREAD
static void max30001_thread(struct max30001_data *data)
{
	while (1) {
		k_sem_take(&data->sem, K_FOREVER);
		max30001_thread_cb(data->dev);
	}
}
#endif

#ifdef CONFIG_MAX30001_TRIGGER_GLOBAL_THREAD
static void max30001_work_cb(struct k_work *work)
{
	struct max30001_data *data =
		CONTAINER_OF(work, struct max30001_data, work);

	max30001_thread_cb(data->dev);
}
#endif

static int max30001_reset(const struct device *dev)
{
	int ret;

	ret = max30001_write_reg(dev, MAX30001_SYNCH, SYNCH_DIN);
	if (ret < 0) {
		return -EIO;
	}

	/* Wait for SYNCH execution complete. */
	k_msleep(10);

	return 0;
}

static int max30001_fifo_reset(const struct device *dev)
{
	int ret;

	ret = max30001_write_reg(dev, MAX30001_FIFO_RST, FIFO_RST_DIN);
	if (ret < 0) {
		return -EIO;
	}

	return 0;
}

static int max30001_configure(const struct device *dev)
{
	const struct max30001_config *cfg = dev->config;
	uint32_t val;

	if (max30001_read_reg(dev, MAX30001_CNFG_ECG, &val) < 0) {
		return -EIO;
	}

	val &= ~CNFG_ECG_ECG_RATE;
	val |= cfg->ecg_sample_rate << ECG_RATE_OFFSET;

	if (max30001_write_reg(dev, MAX30001_CNFG_ECG, val) < 0) {
		return -EIO;
	}

#ifdef CONFIG_MAX30001_LEAD_DETECTION
	if (max30001_read_reg(dev, MAX30001_CNFG_GEN, &val) < 0) {
		return -EIO;
	}

	val &= ~CNFG_GEN_IMAG;
	val |= cfg->lead_off_imag << IMAG_OFFSET;

	val &= ~CNFG_GEN_VTH;
	val |= cfg->lead_off_vth << VTH_OFFSET;

	if (max30001_write_reg(dev, MAX30001_CNFG_GEN, val) < 0) {
		return -EIO;
	}
#endif /* CONFIG_MAX30001_LEAD_DETECTION */

	if (max30001_read_reg(dev, MAX30001_EN_INT, &val) < 0) {
		return -EIO;
	}

	val |= EN_INT_EN_EINT;
#ifdef CONFIG_MAX30001_LEAD_DETECTION
	val |= EN_INT_EN_DCLOFFINT;
	val |= EN_INT_EN_LONINT;
#endif

	if (max30001_write_reg(dev, MAX30001_EN_INT, val) < 0) {
		return -EIO;
	}

	return 0;
}

#ifdef CONFIG_MAX30001_LEAD_DETECTION
static int max30001_trigger_nf_set(const struct device *dev,
				   sensor_trigger_handler_t handler)
{
	struct max30001_data *data = dev->data;
	uint32_t val, original;
	int ret;

	ret = max30001_read_reg(dev, MAX30001_CNFG_GEN, &val);
	if (ret < 0) {
		return -EIO;
	}

	if (handler) {
		if (((val & CNFG_GEN_EN_ULP_LON) == ULP_LON_ENABLE) ||
		    ((val & CNFG_GEN_EN_DCLOFF) == DCLOFF_ENABLE)) {
			data->lead_status_handler = handler;
			return 0;
		}

		if (val & CNFG_GEN_EN_ECG) {
			/* Lead-off detection. */
			val &= ~CNFG_GEN_EN_DCLOFF;
			val |= DCLOFF_ENABLE;
			LOG_INF("Enable DC lead-off detection");
		} else {
			/* Lead-on detection. */
			val &= ~CNFG_GEN_EN_ULP_LON;
			val |= ULP_LON_ENABLE;
			LOG_INF("Enable ultra-low power lead-on detection");
		}

		ret = max30001_write_reg(dev, MAX30001_CNFG_GEN, val);
		if (ret < 0) {
			return -EIO;
		}
		data->lead_status_handler = handler;

		k_msleep(100);
		ret = max30001_read_reg(dev, MAX30001_STATUS, &val);
		if (ret < 0) {
			return -EIO;
		}
		ret = max30001_read_reg(dev, MAX30001_STATUS, &val);
		if (ret < 0) {
			return -EIO;
		}
		LOG_INF("lead-on stat: %x", val);
	} else {
		original = val;

		if ((val & CNFG_GEN_EN_ULP_LON) == ULP_LON_ENABLE) {
			val &= ~CNFG_GEN_EN_ULP_LON;
			val |= ULP_LON_DISABLE;
			LOG_INF("Disable Ultra-low power lead-on detection");
		}
		if ((val & CNFG_GEN_EN_DCLOFF) == DCLOFF_ENABLE) {
			val &= ~CNFG_GEN_EN_DCLOFF;
			val |= DCLOFF_DISABLE;
			LOG_INF("Disable DC lead-off detection");
		}

		if (val == original) {
			return 0;
		}

		/* Disable lead detection. */
		ret = max30001_write_reg(dev, MAX30001_CNFG_GEN, val);
		if (ret < 0) {
			return -EIO;
		}
		data->lead_status_handler = NULL;
	}

	return 0;
}
#endif

static int max30001_trigger_drdy_set(const struct device *dev,
				     sensor_trigger_handler_t handler)
{
	struct max30001_data *data = dev->data;
	uint32_t val;
	int ret;

#ifdef CONFIG_MAX30001_LEAD_DETECTION
	if (handler == NULL) {
		/* Disable lead detection first. */
		ret = max30001_trigger_nf_set(dev, NULL);
		if (ret < 0) {
			return -EIO;
		}
	}
#endif

	ret = max30001_read_reg(dev, MAX30001_CNFG_GEN, &val);
	if (ret < 0) {
		return -EIO;
	}

	if (handler) {

#ifdef CONFIG_MAX30001_LEAD_DETECTION
		/* Lead-on detection running, can not enable ECG sampling. */
		if ((val & CNFG_GEN_EN_ULP_LON) == ULP_LON_ENABLE) {
			LOG_ERR("Enable ECG sampling but Lead-on detection is running");
			return -ENOTSUP;
		}
#endif
		ret = max30001_reset(dev);
		if (ret < 0) {
			return ret;
		}

		/* Enable ECG sampling. */
		val |= CNFG_GEN_EN_ECG;

		ret = max30001_write_reg(dev, MAX30001_CNFG_GEN, val);
		if (ret < 0) {
			return -EIO;
		}
		data->ecg_sample_handler = handler;
	} else {

		/* Disable ECG sampling. */
		val &= ~CNFG_GEN_EN_ECG;
		ret = max30001_write_reg(dev, MAX30001_CNFG_GEN, val);
		if (ret < 0) {
			return -EIO;
		}
		data->ecg_sample_handler = NULL;
	}

	return 0;
}

/*
 * This support two triggers, the SENSOR_TRIG_DATA_READY trigger represent ECG
 * sampling data ready; the SENSOR_TRIG_NEAR_FAR trigger represent lead-on or
 * lead-off event.
 *
 * Lead-on detection can not work with ECG sampling, but lead-off detection
 * relay on ECG sampling. So, trigger set api for SENSOR_TRIG_NEAR_FAR trigger
 * have a dynamic result which depends on SENSOR_TRIG_DATA_READY trigger set or
 * not. If it set, DC lead-off detection will be enable; if not set, ultra-low
 * power lead-on detection will be enable.
 *
 * Set the trigger handler will enable related function, clear the
 * trigger handler will disable related function.
 */
static int max30001_trigger_set(const struct device *dev,
				const struct sensor_trigger *trig,
				sensor_trigger_handler_t handler)
{
	const struct max30001_config *cfg = dev->config;
	int ret;

	gpio_pin_interrupt_configure_dt(&cfg->intb, GPIO_INT_DISABLE);

	if (trig->type == SENSOR_TRIG_DATA_READY) {
		ret = max30001_trigger_drdy_set(dev, handler);
#ifdef CONFIG_MAX30001_LEAD_DETECTION
	} else if (trig->type == SENSOR_TRIG_NEAR_FAR) {
		ret = max30001_trigger_nf_set(dev, handler);
#endif
	} else {
		ret = -EINVAL;
	}

	gpio_pin_interrupt_configure_dt(&cfg->intb, GPIO_INT_EDGE_TO_INACTIVE);

	return ret;
}

/**
 * @brief Fetch the ECG sample.
 *
 * @param dev      Pointer to the sensor device
 * @param chan     The channel to read
 *
 * @return Negative if failure, others for success.
 * @retval 1       There still have samples in ECG_FIFO.
 * @retval 0       There doesn't have others sample in ECG_FIFO.
 * @retval -EPERM  FIFO memory is empty, sample fetch do nothing.
 * @retval -ENOMEM FIFO memory is overflow, automatic apply reset process.
 * @retval -EIO    I/O error.
 * @retval -EINVAL Invalid parameter.
 */
static int max30001_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	struct max30001_data *data = dev->data;
	uint32_t val;

	if (chan != SENSOR_CHAN_VOLTAGE) {
		return -EINVAL;
	}

	if (max30001_read_reg(dev, MAX30001_ECG_FIFO, &val) < 0) {
		return -EIO;
	}

	switch (val & ECG_FIFO_ETAG) {
	case ECG_FIFO_ETAG_VALID:
		/* Extract ECG sample from ECG_FIFO. */
		data->ecg_sample = (int)(val & ECG_FIFO_SAMPLE) << 8;
		return 1;
	case ECG_FIFO_ETAG_EOF:
		/* Extract ECG sample from ECG_FIFO. */
		data->ecg_sample = (int)(val & ECG_FIFO_SAMPLE) << 8;
		return 0;
	case ECG_FIFO_ETAG_EMPTY:
		/* ECG_FIFO is empty, sample fetch do nothing. */
		return -EPERM;
	case ECG_FIFO_ETAG_OVERFLOW:
		/* ECG_FIFO overflow, reset will automatic applied. */
		max30001_fifo_reset(dev);
		return -ENOMEM;
	default:
		return -EINVAL;
	}
}

static int max30001_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	struct max30001_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
		val->val1 = data->ecg_sample;
		break;
#ifdef CONFIG_MAX30001_LEAD_DETECTION
	case SENSOR_CHAN_PROX:
		val->val1 = (int)data->lead_status;
		break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct sensor_driver_api max30001_driver_api = {
	.trigger_set = max30001_trigger_set,
	.sample_fetch = max30001_sample_fetch,
	.channel_get = max30001_channel_get,
};

static int max30001_init(const struct device *dev)
{
	struct max30001_data *data = dev->data;
	const struct max30001_config *cfg = dev->config;
	int ret;

	data->dev = dev;

	gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
	gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_4);
	gpio_bit_reset(GPIOB, GPIO_PIN_4);

	k_msleep(5);

	if (!spi_is_ready(&cfg->bus)) {
		LOG_ERR("SPI bus %s not ready", cfg->bus.bus->name);
		return -ENODEV;
	}

	ret = max30001_configure(dev);
	if (ret < 0) {
		LOG_ERR("Failed to configure max30001");
		return ret;
	}

#ifdef CONFIG_MAX30001_TRIGGER_OWN_THREAD
	k_sem_init(&data->sem, 0, 1);

	k_thread_create(&data->thread, data->stack,
			      CONFIG_MAX30001_THREAD_STACK_SIZE,
			      (k_thread_entry_t)max30001_thread,
			      data, NULL, NULL,
			      CONFIG_MAX30001_THREAD_PRIORITY,
			      0, K_NO_WAIT);
#elif defined(CONFIG_MAX30001_TRIGGER_GLOBAL_THREAD)
	data->work.handler = max30001_work_cb;
#endif

	ret = gpio_pin_configure_dt(&cfg->intb, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure intb gpio");
		return ret;
	}

	gpio_init_callback(&data->intb_cb, max30001_intb_callback,
			   BIT(cfg->intb.pin));

	ret = gpio_add_callback(cfg->intb.port, &data->intb_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add intb callback");
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->intb, GPIO_INT_EDGE_TO_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure intb interrupt");
		return ret;
	}

	return 0;
}

#ifdef CONFIG_MAX30001_LEAD_DETECTION
#define LEAD_DETECTION_CFG_INIT(n)						\
		.lead_off_imag = DT_INST_PROP(n, lead_off_imag),		\
		.lead_off_vth = DT_INST_PROP(n, lead_off_vth),
#else
#define LEAD_DETECTION_CFG_INIT(n)
#endif

#define MAX30001_INIT(n)							\
	static const struct max30001_config max30001_cfg_##n = {		\
		.bus = SPI_DT_SPEC_INST_GET(n,					\
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |			\
			SPI_WORD_SET(8), 0),					\
		.intb = GPIO_DT_SPEC_INST_GET(n, intb_gpios),			\
		.ecg_sample_rate =  DT_INST_PROP(n, ecg_sample_rate),		\
		LEAD_DETECTION_CFG_INIT(n)					\
	};									\
	static struct max30001_data max30001_data_##n;				\
	DEVICE_DT_INST_DEFINE(n, &max30001_init, NULL,				\
			      &max30001_data_##n, &max30001_cfg_##n,		\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,		\
			      &max30001_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX30001_INIT)
