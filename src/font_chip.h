/*
 * Copyright (c) 2026 Tomimi
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __FONT_CHIP_H
#define __FONT_CHIP_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/display.h>

typedef enum {
	ZK_SIZE_5X7 = 1,
	ZK_SIZE_7X8 = 2,
	ZK_SIZE_6X12 = 3,
	ZK_SIZE_8X16 = 4,
	ZK_SIZE_12X24 = 5,
	ZK_SIZE_16X32 = 6,
} zk_asc_size_t;

typedef enum {
	ZK_CH_12X12 = 1,
	ZK_CH_16X16 = 2,
	ZK_CH_24X24 = 3,
	ZK_CH_32X32 = 4,
} zk_chinese_size_t;

int font_chip_init(const struct device *display);
void font_chip_draw_char_asc(uint16_t x, uint16_t y, char c, zk_asc_size_t size, uint16_t fc, uint16_t bc);
void font_chip_draw_string_asc(uint16_t x, uint16_t y, const char *str, zk_asc_size_t size, uint16_t fc, uint16_t bc);
void font_chip_draw_chinese_gb2312(uint16_t x, uint16_t y, uint8_t msb, uint8_t lsb, zk_chinese_size_t size, uint16_t fc, uint16_t bc);

#endif /* __FONT_CHIP_H */
