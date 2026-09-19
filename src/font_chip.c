/*
 * Copyright (c) 2026 Tomimi
 * SPDX-License-Identifier: Apache-2.0
 */

#include "font_chip.h"
#include <errno.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(font_chip_final, LOG_LEVEL_INF);

#define DISPLAY_WIDTH  160
#define DISPLAY_HEIGHT 128

static const struct device *display_dev;
static const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(lpspi0));
static const struct gpio_dt_spec font_cs_spec = GPIO_DT_SPEC_GET(DT_ALIAS(font_cs), gpios);

#if DT_NODE_EXISTS(DT_ALIAS(lcd_backlight))
static const struct gpio_dt_spec blk_spec = GPIO_DT_SPEC_GET(DT_ALIAS(lcd_backlight), gpios);
#endif

static uint8_t font_buf[128];
static uint16_t pixel_buf[32 * 32];

static const struct spi_config spi_cfg = {
	.frequency = 10000000,
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.slave = 0,
	/* CS is shared with the LCD and is driven explicitly around ROM reads. */
};

int font_chip_init(const struct device *display)
{
	if (display == NULL || !device_is_ready(display)) {
		LOG_ERR("Display device is not ready");
		return -ENODEV;
	}
	if (!device_is_ready(spi_dev)) {
		LOG_ERR("Font ROM SPI device is not ready");
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&font_cs_spec)) {
		LOG_ERR("Font ROM CS GPIO is not ready");
		return -ENODEV;
	}
	int ret = gpio_pin_configure_dt(&font_cs_spec, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Cannot configure font ROM CS (%d)", ret);
		return ret;
	}

	display_dev = display;

#if DT_NODE_EXISTS(DT_ALIAS(lcd_backlight))
	if (!gpio_is_ready_dt(&blk_spec)) {
		LOG_ERR("LCD backlight GPIO is not ready");
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&blk_spec, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Cannot configure LCD backlight (%d)", ret);
		return ret;
	}
#endif

	return 0;
}

static int get_n_bytes_from_rom(uint32_t addr, uint8_t *buf, uint16_t len)
{
	uint8_t cmd[4] = {
		0x03,
		(uint8_t)((addr >> 16) & 0xFF),
		(uint8_t)((addr >> 8) & 0xFF),
		(uint8_t)(addr & 0xFF),
	};
	const struct spi_buf tx_bufs[] = {
		{ .buf = cmd, .len = sizeof(cmd) },
	};
	const struct spi_buf rx_bufs[] = {
		{ .buf = NULL, .len = sizeof(cmd) },
		{ .buf = buf, .len = len },
	};
	const struct spi_buf_set tx = { .buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs) };
	const struct spi_buf_set rx = { .buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs) };

	/* The board routes the shared physical CS high through an inverter to the ROM. */
	int ret = gpio_pin_set_dt(&font_cs_spec, 1);
	if (ret < 0) {
		return ret;
	}
	ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
	int deselect_ret = gpio_pin_set_dt(&font_cs_spec, 0);
	return ret < 0 ? ret : deselect_ret;
}

static bool display_region_is_valid(uint16_t x, uint16_t y, uint8_t width, uint8_t height)
{
	return display_dev != NULL && x < DISPLAY_WIDTH && y < DISPLAY_HEIGHT &&
	       width <= DISPLAY_WIDTH - x && height <= DISPLAY_HEIGHT - y;
}

static void draw_pixels(uint16_t x, uint16_t y, uint8_t width, uint8_t height,
			uint16_t fc, uint16_t bc)
{
	if (!display_region_is_valid(x, y, width, height)) {
		return;
	}

	struct display_buffer_descriptor desc = {
		.buf_size = (size_t)width * height * sizeof(uint16_t),
		.width = width,
		.height = height,
		.pitch = width,
	};
	int ret = display_write(display_dev, x, y, &desc, pixel_buf);
	if (ret < 0) {
		LOG_ERR("Display write failed (%d)", ret);
	}
	ARG_UNUSED(fc);
	ARG_UNUSED(bc);
}

void font_chip_draw_char_asc(uint16_t x, uint16_t y, char c, zk_asc_size_t size,
			     uint16_t fc, uint16_t bc)
{
	if (c < 0x20 || c > 0x7E) {
		return;
	}

	uint32_t base_addr;
	uint8_t bytes_per_char;
	uint8_t width;
	uint8_t height;

	switch (size) {
	case ZK_SIZE_5X7:
		base_addr = 0x1DDF80; bytes_per_char = 8; width = 6; height = 8; break;
	case ZK_SIZE_7X8:
		base_addr = 0x1DE280; bytes_per_char = 8; width = 8; height = 8; break;
	case ZK_SIZE_6X12:
		base_addr = 0x1DBE00; bytes_per_char = 12; width = 6; height = 12; break;
	case ZK_SIZE_8X16:
		base_addr = 0x1DD780; bytes_per_char = 16; width = 8; height = 16; break;
	case ZK_SIZE_12X24:
		base_addr = 0x1DFF00; bytes_per_char = 48; width = 12; height = 24; break;
	case ZK_SIZE_16X32:
		base_addr = 0x1E5A50; bytes_per_char = 64; width = 16; height = 32; break;
	default:
		return;
	}

	if (!display_region_is_valid(x, y, width, height) ||
	    get_n_bytes_from_rom(base_addr + (uint32_t)(c - 0x20) * bytes_per_char,
				font_buf, bytes_per_char) < 0) {
		return;
	}

	uint16_t p_idx = 0;
	uint8_t row_bytes = (width + 7U) / 8U;
	for (uint8_t row = 0; row < height; row++) {
		for (uint8_t col = 0; col < width; col++) {
			uint8_t byte_idx = row * row_bytes + col / 8U;
			uint8_t bit_idx = 7U - (col % 8U);
			uint16_t color = (font_buf[byte_idx] & BIT(bit_idx)) ? fc : bc;
			pixel_buf[p_idx++] = (uint16_t)((color >> 8) | (color << 8));
		}
	}
	draw_pixels(x, y, width, height, fc, bc);
}

static bool gb2312_address(uint8_t msb, uint8_t lsb, zk_chinese_size_t size,
			   uint32_t *address, uint8_t *bytes, uint8_t *width)
{
	if (lsb < 0xA1 || lsb > 0xFE ||
	    !((msb >= 0xA1 && msb <= 0xA9) || (msb >= 0xB0 && msb <= 0xF7))) {
		return false;
	}

	uint32_t index;
	if (msb <= 0xA9) {
		index = (uint32_t)(msb - 0xA1) * 94U + (lsb - 0xA1);
	} else {
		index = (uint32_t)(msb - 0xB0) * 94U + (lsb - 0xA1) + 846U;
	}

	switch (size) {
	case ZK_CH_12X12:
		*bytes = 24; *width = 12; *address = index * 24U; return true;
	case ZK_CH_16X16:
		*bytes = 32; *width = 16; *address = 0x2C9D0U + index * 32U; return true;
	case ZK_CH_24X24:
		*bytes = 72; *width = 24; *address = 0x68190U + index * 72U; return true;
	case ZK_CH_32X32:
		*bytes = 128; *width = 32; *address = 0xEDF00U + index * 128U; return true;
	default:
		return false;
	}
}

static void draw_char_cn(uint16_t x, uint16_t y, uint8_t msb, uint8_t lsb,
			 zk_chinese_size_t size, uint16_t fc, uint16_t bc)
{
	uint32_t address;
	uint8_t bytes;
	uint8_t width;
	if (!gb2312_address(msb, lsb, size, &address, &bytes, &width) ||
	    !display_region_is_valid(x, y, width, width) ||
	    get_n_bytes_from_rom(address, font_buf, bytes) < 0) {
		return;
	}

	uint16_t p_idx = 0;
	uint8_t row_bytes = (width + 7U) / 8U;
	for (uint8_t row = 0; row < width; row++) {
		for (uint8_t col = 0; col < width; col++) {
			uint8_t byte_idx = row * row_bytes + col / 8U;
			uint8_t bit_idx = 7U - (col % 8U);
			uint16_t color = (font_buf[byte_idx] & BIT(bit_idx)) ? fc : bc;
			pixel_buf[p_idx++] = (uint16_t)((color >> 8) | (color << 8));
		}
	}
	draw_pixels(x, y, width, width, fc, bc);
}

void font_chip_draw_chinese_gb2312(uint16_t x, uint16_t y, uint8_t msb, uint8_t lsb,
				   zk_chinese_size_t size, uint16_t fc, uint16_t bc)
{
	draw_char_cn(x, y, msb, lsb, size, fc, bc);
}

void font_chip_draw_string_asc(uint16_t x, uint16_t y, const char *str,
			       zk_asc_size_t size, uint16_t fc, uint16_t bc)
{
	if (str == NULL) {
		return;
	}

	uint8_t char_width;
	uint8_t char_height;
	switch (size) {
	case ZK_SIZE_5X7: char_width = 6; char_height = 8; break;
	case ZK_SIZE_7X8: char_width = 8; char_height = 8; break;
	case ZK_SIZE_6X12: char_width = 6; char_height = 12; break;
	case ZK_SIZE_8X16: char_width = 8; char_height = 16; break;
	case ZK_SIZE_12X24: char_width = 12; char_height = 24; break;
	case ZK_SIZE_16X32: char_width = 16; char_height = 32; break;
	default: return;
	}

	uint16_t cur_x = x;
	uint16_t cur_y = y;
	while (*str != '\0') {
		if (*str == '\n') {
			cur_x = x;
			cur_y += char_height;
			str++;
			continue;
		}
		if (*str < 0x20 || *str > 0x7E) {
			str++;
			continue;
		}
		if (cur_x + char_width > DISPLAY_WIDTH || cur_y + char_height > DISPLAY_HEIGHT) {
			break;
		}
		font_chip_draw_char_asc(cur_x, cur_y, *str++, size, fc, bc);
		cur_x += char_width;
	}
}
