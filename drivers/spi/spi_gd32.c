/*
 * Copyright (c) 2021 BrainCo Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_spi

#include <drivers/pinctrl.h>
#include <drivers/spi.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(spi_gd32);

#include "spi_context.h"

/* SPI error status mask */
#define SPI_GD32_ERR_MASK	(SPI_STAT_RXORERR | SPI_STAT_CONFERR | SPI_STAT_CRCERR)

struct spi_gd32_config {
	uint32_t reg;
	uint32_t rcu_periph_clock;
	const struct pinctrl_dev_config *pcfg;
};

struct spi_gd32_data {
	struct spi_context ctx;
};

static int spi_gd32_get_err(uint32_t spi)
{
	uint32_t stat = SPI_STAT(spi);

	if (stat & SPI_GD32_ERR_MASK) {
		LOG_WRN("spi%u error status detected, err = %u",
			spi, stat & (uint32_t)SPI_GD32_ERR_MASK);

		return -EIO;
	}

	return 0;
}

static bool spi_gd32_transfer_ongoing(struct spi_gd32_data *data)
{
	return spi_context_tx_on(&data->ctx) ||
	       spi_context_rx_on(&data->ctx);
}

static int spi_gd32_configure(const struct device *dev,
			      const struct spi_config *config)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	spi_parameter_struct spi_parameter;
	uint32_t cpol = 0U, cpha = 0U;

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	spi_disable(cfg->reg);

	spi_parameter.device_mode = SPI_MASTER;
	spi_parameter.trans_mode = SPI_TRANSMODE_FULLDUPLEX;

	if (SPI_WORD_SIZE_GET(config->operation) == 8) {
		spi_parameter.frame_size = SPI_FRAMESIZE_8BIT;
	} else {
		spi_parameter.frame_size = SPI_FRAMESIZE_16BIT;
	}

	if (config->cs) {
		spi_parameter.nss = SPI_NSS_SOFT;
	} else {
		spi_parameter.nss = SPI_NSS_HARD;
		spi_nss_output_enable(cfg->reg);
	}

	if (config->operation & SPI_TRANSFER_LSB) {
		spi_parameter.endian = SPI_ENDIAN_LSB;
	} else {
		spi_parameter.endian = SPI_ENDIAN_MSB;
	}

	if (config->operation & SPI_MODE_CPOL) {
		cpol = SPI_CTL0_CKPL;
	}

	if (config->operation & SPI_MODE_CPHA) {
		cpha = SPI_CTL0_CKPH;
	}

	spi_parameter.clock_polarity_phase = cpol | cpha;

	/* [TODO] implement spi frequency select */
	spi_parameter.prescale = SPI_PSC_32;

	spi_init(cfg->reg, &spi_parameter);

	data->ctx.config = config;

	return 0;
}

static int spi_gd32_frame_exchange(struct spi_gd32_data *data, uint32_t spi)
{
	struct spi_context *ctx = &data->ctx;
	uint16_t tx_frame = 0U, rx_frame = 0U;

	if (SPI_WORD_SIZE_GET(ctx->config->operation) == 8) {
		if (spi_context_tx_buf_on(ctx)) {
			tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
		}
		spi_i2s_data_transmit(spi, tx_frame);

		spi_context_update_tx(ctx, 1, 1);
	} else {
		if (spi_context_tx_buf_on(ctx)) {
			tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
		}
		spi_i2s_data_transmit(spi, tx_frame);

		spi_context_update_tx(ctx, 2, 1);
	}

	while (!spi_i2s_flag_get(spi, SPI_FLAG_RBNE)) {
		/* NOP */
	}

	if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
		rx_frame = spi_i2s_data_receive(spi);
		if (spi_context_rx_buf_on(ctx)) {
			UNALIGNED_PUT(rx_frame, (uint8_t *)data->ctx.rx_buf);
		}

		spi_context_update_rx(ctx, 1, 1);
	} else {
		rx_frame = spi_i2s_data_receive(spi);
		if (spi_context_rx_buf_on(ctx)) {
			UNALIGNED_PUT(rx_frame, (uint16_t *)data->ctx.rx_buf);
		}

		spi_context_update_rx(ctx, 2, 1);
	}

	return spi_gd32_get_err(spi);
}

static int spi_gd32_transceive(const struct device *dev,
			       const struct spi_config *config,
			       const struct spi_buf_set *tx_bufs,
			       const struct spi_buf_set *rx_bufs)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	int ret;

	spi_context_lock(&data->ctx, false, NULL, config);

	ret = spi_gd32_configure(dev, config);
	if (ret) {
		goto out;
	}

	spi_enable(cfg->reg);

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);

	do {
		ret = spi_gd32_frame_exchange(data, cfg->reg);
	} while (!ret && spi_gd32_transfer_ongoing(data));

	spi_context_cs_control(&data->ctx, false);

	spi_disable(cfg->reg);
out:
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_gd32_release(const struct device *dev,
			    const struct spi_config *config)
{
	struct spi_gd32_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static struct spi_driver_api spi_gd32_driver_api = {
	.transceive = spi_gd32_transceive,
	.release = spi_gd32_release
};

int spi_gd32_init(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	int ret;

	rcu_periph_clock_enable(cfg->rcu_periph_clock);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to apply pinctrl state");
		return ret;
	}

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		return ret;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define GD32_SPI_INIT(idx)							\
	PINCTRL_DT_INST_DEFINE(idx)						\
	static struct spi_gd32_data spi_gd32_data_##idx = {			\
		SPI_CONTEXT_INIT_LOCK(spi_gd32_data_##idx, ctx),		\
		SPI_CONTEXT_INIT_SYNC(spi_gd32_data_##idx, ctx),		\
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(idx), ctx)		\
	};									\
	static struct spi_gd32_config spi_gd32_config_##idx = {			\
		.reg = DT_INST_REG_ADDR(idx),					\
		.rcu_periph_clock = DT_INST_PROP(idx, rcu_periph_clock),	\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),			\
	};									\
	DEVICE_DT_INST_DEFINE(idx, &spi_gd32_init,				\
			      NULL,						\
			      &spi_gd32_data_##idx,				\
			      &spi_gd32_config_##idx,				\
			      POST_KERNEL,					\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			      &spi_gd32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GD32_SPI_INIT)
