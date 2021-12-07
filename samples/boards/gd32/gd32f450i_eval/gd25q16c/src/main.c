/*
 * Copyright (c) 2021 BrainCo Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/spi.h>

#define	SPI DT_NODELABEL(spi5)

#define ID_0 0xc8
#define ID_1 0x40
#define ID_2 0x15

static uint8_t __aligned(4) flash_gd25q16c_cmd_buf[8];
static uint8_t __aligned(4) flash_gd25q16c_data_buf[32];

static const struct spi_cs_control cs_control = {
	.gpio = GPIO_DT_SPEC_GET_BY_IDX(SPI, cs_gpios, 0),
	.delay = 0U
};

static const struct spi_config gd25q16c_spi_cfg = {
	.frequency = 1000000, /* 1 MHz */
	.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
	.slave = 0,
	.cs = &cs_control
};

static int flash_gd25q16c_get_id(const struct device *spi)
{
	struct spi_buf_set tx_bufs, rx_bufs;
	struct spi_buf tx_buf[1];
	struct spi_buf rx_buf[2];
	uint8_t *cmd_buf = flash_gd25q16c_cmd_buf;
	uint8_t *data_buf = flash_gd25q16c_data_buf;
	int ret;

	/* Read Identification */
	cmd_buf[0] = 0x9F;

	tx_buf[0].buf = cmd_buf;
	tx_buf[0].len = 1;

	rx_buf[0].buf = NULL;
	rx_buf[0].len = 1;
	rx_buf[1].buf = data_buf;
	rx_buf[1].len = 3;

	tx_bufs.buffers = tx_buf;
	tx_bufs.count = 1;

	rx_bufs.buffers = rx_buf;
	rx_bufs.count = 2;

	ret = spi_transceive(spi, &gd25q16c_spi_cfg, &tx_bufs, &rx_bufs);

	if (data_buf[0] == ID_0 &&
	    data_buf[1] == ID_1 &&
	    data_buf[2] == ID_2) {
		printk("Read gd25q16c ID success, ID: 0x%02x%02x%02x\n",
		       data_buf[0], data_buf[1], data_buf[2]);
	} else {
		ret = -1;
	}

	return ret;
}

void main(void)
{
	const struct device *spi;
	int ret;

	spi = device_get_binding(DT_LABEL(SPI));
	if (!device_is_ready(spi)) {
		printk("spi device not ready.\n");
		return;
	}

	ret = flash_gd25q16c_get_id(spi);
	if (ret) {
		printk("Get gd25q16c ID failed.\n");
	}
}
