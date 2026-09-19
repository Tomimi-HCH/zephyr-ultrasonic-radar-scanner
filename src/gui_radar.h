/*
 * Copyright (c) 2026 Tomimi
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __GUI_RADAR_H
#define __GUI_RADAR_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>

#define RADAR_CENTER_X 80
#define RADAR_CENTER_Y 120
#define RADAR_MAX_R    78

#define COLOR_RADAR_BG    0x0000
#define COLOR_RADAR_GRID  0x03E0
#define COLOR_RADAR_BEAM  0x07E0
#define COLOR_RADAR_POINT 0xF800
#define COLOR_RADAR_TEXT  0xFFFF
#define COLOR_RADAR_BOX   0xFFE0

int gui_radar_init(const struct device *display);
void gui_radar_clear_screen(void);
void gui_radar_reset_view(void);
void gui_radar_draw_static_frame(void);
void gui_radar_update_scan_line(uint8_t angle, uint16_t dist_mm, uint16_t max_range_mm);
void gui_radar_draw_pixel(int16_t x, int16_t y, uint16_t color);
void gui_radar_draw_object_box(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);

#endif /* __GUI_RADAR_H */
