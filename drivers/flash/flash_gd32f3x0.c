/*
 * Copyright (c) 2022 BrainCo Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <soc.h>
#include "flash_gd32.h"

#include <logging/log.h>
LOG_MODULE_DECLARE(flash_gd32);

#define GD32F3X0_FLASH_TIMEOUT 300

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_gd32f3x0_layout = {
	.pages_count = SOC_NV_FLASH_SIZE /
			SOC_NV_FLASH_ERASE_BLOCK_SIZE,
	.pages_size = SOC_NV_FLASH_ERASE_BLOCK_SIZE,
};
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static void flash_gd32_err_handler(void)
{
	if (FMC_STAT & FMC_STAT_PGERR) {
		FMC_STAT &= ~FMC_STAT_PGERR;
		LOG_ERR("PGERR: erasing/programming on protected pages.");
	}

	if (FMC_STAT & FMC_STAT_WPERR) {
		FMC_STAT &= ~FMC_STAT_WPERR;
		LOG_ERR("WPERR: programming to not 0xFF position.");
	}
}

int flash_gd32_programming(off_t offset, const void *data, size_t len)
{
	flash_prog_t *src = (flash_prog_t *)data;
	flash_prog_t *dst = (flash_prog_t *)SOC_NV_FLASH_ADDR + offset;

	if (FMC_STAT & FMC_STAT_BUSY) {
		return -EBUSY;
	}

	/* Set the PG bit to enable flash programming */
	FMC_CTL |= FMC_CTL_PG;

	for (size_t i = 0; i < (len / sizeof(flash_prog_t)); i++) {
		*dst++ = *src++;
	}

	/* Wait for the sector erase complete. */
	while (FMC_STAT & FMC_STAT_BUSY) {
		/* Nop */
	}

	/* Check the operation executed successfully. */
	if ((FMC_STAT & FMC_STAT_ENDF) != 0) {
		flash_gd32_err_handler();
		return -EIO;
	}
	FMC_STAT &= ~FMC_STAT_ENDF;

	/* Clear the PG bit to disable flash programming */
	FMC_CTL &= ~FMC_CTL_PG;

	return 0;
}

int flash_gd32_page_erase(uint32_t page)
{
	int64_t timeout_time = k_uptime_get() + GD32F3X0_FLASH_TIMEOUT;

	if (FMC_STAT & FMC_STAT_BUSY) {
		return -EBUSY;
	}

	FMC_ADDR = page;

	/* Set page erase command bit. */
	FMC_CTL |= FMC_CTL_PER;

	/* Send the erase command to FMC. */
	FMC_CTL |= FMC_CTL_START;

	/* Wait for the page erase complete. */
	while (FMC_STAT & FMC_STAT_BUSY) {
		if (k_uptime_get() >= timeout_time) {
			return -EIO;
		}
	}

	/* Check the operation executed successfully. */
	if ((FMC_STAT & FMC_STAT_ENDF) != 0) {
		flash_gd32_err_handler();
		return -EIO;
	}
	FMC_STAT &= ~FMC_STAT_ENDF;

	/* Verify the erased page is correct. */
	if (page != FMC_ADDR) {
		return -EIO;
	}

	return 0;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
void flash_gd32_pages_layout(const struct device *dev,
			     const struct flash_pages_layout **layout,
			     size_t *layout_size)
{
	ARG_UNUSED(dev);

	*layout = &flash_gd32f3x0_layout;
	*layout_size = 1U;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
