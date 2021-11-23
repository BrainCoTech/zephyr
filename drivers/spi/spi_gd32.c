/*
 * Copyright (c) 2021 BrainCo Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gd_gd32_spi

#include <errno.h>
#include <kernel.h>
#include <drivers/pinctrl.h>
#include <drivers/spi.h>
#include <soc.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(spi_gd32);

#include "spi_context.h"

/* SPI error status mask. */
#define SPI_GD32_ERR_MASK	(SPI_STAT_RXORERR | SPI_STAT_CONFERR | SPI_STAT_CRCERR)

struct spi_gd32_config {
	uint32_t reg;
	uint32_t rcu_periph_clock;
	const struct pinctrl_dev_config *pcfg;
};

struct spi_gd32_data {
	struct spi_context ctx;
};

static int spi_gd32_get_err(const struct spi_gd32_config *cfg)
{
	uint32_t stat = SPI_STAT(cfg->reg);

	if (stat & SPI_GD32_ERR_MASK) {
		LOG_ERR("spi%u error status detected, err = %u",
			cfg->reg, stat & (uint32_t)SPI_GD32_ERR_MASK);

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

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}

	/* Before configure, disable spi first. */
	I2C_CTL0(cfg->reg) &= ~SPI_CTL0_SPIEN;

	/* Default to master mode. */
	I2C_CTL0(cfg->reg) |= SPI_MASTER;
	/* Default to full duplex transfer mode. */
	I2C_CTL0(cfg->reg) &= ~SPI_TRANSMODE_BDTRANSMIT;

	if (SPI_WORD_SIZE_GET(config->operation) == 8) {
		I2C_CTL0(cfg->reg) |= SPI_FRAMESIZE_8BIT;
	} else {
		I2C_CTL0(cfg->reg) |= SPI_FRAMESIZE_16BIT;
	}

	/* Reset to hardware NSS mode. */
	I2C_CTL0(cfg->reg) &= ~SPI_CTL0_SWNSSEN;
	if (config->cs != NULL) {
		I2C_CTL0(cfg->reg) |= SPI_CTL0_SWNSSEN;
	} else {
		/*
		 * For signle master env,
		 * hardware NSS mode also need to set the NSSDRV bit.
		 */
		SPI_CTL1(cfg->reg) |= SPI_CTL1_NSSDRV;
	}

	/* Reset to MSB first mode. */
	I2C_CTL0(cfg->reg) &= ~SPI_CTL0_LF;
	if (config->operation & SPI_TRANSFER_LSB) {
		I2C_CTL0(cfg->reg) |= SPI_CTL0_LF;
	}

	I2C_CTL0(cfg->reg) &= ~SPI_CTL0_CKPL;
	if (config->operation & SPI_MODE_CPOL) {
		I2C_CTL0(cfg->reg) |= SPI_CTL0_CKPL;
	}

	I2C_CTL0(cfg->reg) &= ~SPI_CTL0_CKPH;
	if (config->operation & SPI_MODE_CPHA) {
		I2C_CTL0(cfg->reg) |= SPI_CTL0_CKPH;
	}

	/* Default spi clock prescaler to PCLK/32. */
	I2C_CTL0(cfg->reg) &= ~SPI_CTL0_PSC;
	I2C_CTL0(cfg->reg) |= SPI_PSC_32;

	data->ctx.config = config;

	return 0;
}

static int spi_gd32_frame_exchange(const struct device *dev)
{
	struct spi_gd32_data *data = dev->data;
	const struct spi_gd32_config *cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	uint16_t tx_frame = 0U, rx_frame = 0U;

	if (SPI_WORD_SIZE_GET(ctx->config->operation) == 8) {
		if (spi_context_tx_buf_on(ctx)) {
			tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
		}
		/* For 8 bits mode, spi will forced SPI_DATA[15:8] to 0. */
		SPI_DATA(cfg->reg) = tx_frame;

		spi_context_update_tx(ctx, 1, 1);
	} else {
		if (spi_context_tx_buf_on(ctx)) {
			tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
		}
		SPI_DATA(cfg->reg) = tx_frame;

		spi_context_update_tx(ctx, 2, 1);
	}

	while ((SPI_STAT(cfg->reg) & SPI_STAT_RBNE) == 0) {
		/* NOP */
	}

	if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
		/* For 8 bits mode, spi will forced SPI_DATA[15:8] to 0. */
		rx_frame = SPI_DATA(cfg->reg);
		if (spi_context_rx_buf_on(ctx)) {
			UNALIGNED_PUT(rx_frame, (uint8_t *)data->ctx.rx_buf);
		}

		spi_context_update_rx(ctx, 1, 1);
	} else {
		rx_frame = SPI_DATA(cfg->reg);
		if (spi_context_rx_buf_on(ctx)) {
			UNALIGNED_PUT(rx_frame, (uint16_t *)data->ctx.rx_buf);
		}

		spi_context_update_rx(ctx, 2, 1);
	}

	return spi_gd32_get_err(cfg);
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
	if (ret < 0) {
		goto error;
	}

	/* Enable this spi peripheral. */
	I2C_CTL0(cfg->reg) |= SPI_CTL0_SPIEN;

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);

	do {
		ret = spi_gd32_frame_exchange(dev);
		if (ret < 0) {
			break;
		}
	} while (spi_gd32_transfer_ongoing(data));

	spi_context_cs_control(&data->ctx, false);

	/* Disable this spi peripheral. */
	I2C_CTL0(cfg->reg) &= ~SPI_CTL0_SPIEN;

error:
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
	PINCTRL_DT_INST_DEFINE(idx);						\
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
	DEVICE_DT_INST_DEFINE(idx, &spi_gd32_init, NULL, &spi_gd32_data_##idx,	\
			      &spi_gd32_config_##idx, POST_KERNEL,		\
			      CONFIG_SPI_INIT_PRIORITY, &spi_gd32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GD32_SPI_INIT)
