/*
 * Copyright (c) 2026 Tomimi
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gui_radar.h"
#include <stdlib.h>

static const struct device *disp_dev;

static const int16_t sin_lut_q15[181] = {
	0, 571, 1143, 1714, 2285, 2855, 3425, 3993, 4560, 5125,
	5689, 6252, 6812, 7370, 7927, 8480, 9031, 9580, 10125, 10667,
	11206, 11742, 12274, 12803, 13327, 13848, 14364, 14875, 15383, 15885,
	16383, 16876, 17363, 17846, 18323, 18794, 19259, 19719, 20172, 20620,
	21062, 21497, 21925, 22347, 22762, 23169, 23570, 23964, 24350, 24729,
	25101, 25465, 25821, 26169, 26509, 26841, 27165, 27481, 27788, 27988,
	28377, 28659, 28931, 29196, 29451, 29697, 29934, 30162, 30381, 30591,
	30791, 30982, 31163, 31335, 31497, 31650, 31792, 31925, 32047, 32160,
	32262, 32355, 32437, 32509, 32571, 32623, 32665, 32696, 32717, 32728,
	32767, 32728, 32717, 32696, 32665, 32623, 32571, 32509, 32437, 32355,
	32262, 32160, 32047, 31925, 31792, 31650, 31497, 31335, 31163, 30982,
	30791, 30591, 30381, 30162, 29934, 29697, 29451, 29196, 28931, 28659,
	28377, 27988, 27788, 27481, 27165, 26841, 26509, 26169, 25821, 25465,
	25101, 24729, 24350, 23964, 23570, 23169, 22762, 22347, 21925, 21497,
	21062, 20620, 20172, 19719, 19259, 18794, 18323, 17846, 17363, 16876,
	16383, 15885, 15383, 14875, 14364, 13848, 13327, 12803, 12274, 11742,
	11206, 10667, 10125, 9580, 9031, 8480, 7927, 7370, 6812, 6252,
	5689, 5125, 4560, 3993, 3425, 2855, 2285, 1714, 1143, 571, 0
};

static const int16_t cos_lut_q15[181] = {
	32767, 32728, 32717, 32696, 32665, 32623, 32571, 32509, 32437, 32355,
	32262, 32160, 32047, 31925, 31792, 31650, 31497, 31335, 31163, 30982,
	30791, 30591, 30381, 30162, 29934, 29697, 29451, 29196, 28931, 28659,
	28377, 27988, 27788, 27481, 27165, 26841, 26509, 26169, 25821, 25465,
	25101, 24729, 24350, 23964, 23570, 23169, 22762, 22347, 21925, 21497,
	21062, 20620, 20172, 19719, 19259, 18794, 18323, 17846, 17363, 16876,
	16383, 15885, 15383, 14875, 14364, 13848, 13327, 12803, 12274, 11742,
	11206, 10667, 10125, 9580, 9031, 8480, 7927, 7370, 6812, 6252,
	5689, 5125, 4560, 3993, 3425, 2855, 2285, 1714, 1143, 571,
	0, -571, -1143, -1714, -2285, -2855, -3425, -3993, -4560, -5125,
	-5689, -6252, -6812, -7370, -7927, -8480, -9031, -9580, -10125, -10667,
	-11206, -11742, -12274, -12803, -13327, -13848, -14364, -14875, -15383, -15885,
	-16383, -16876, -17363, -17846, -18323, -18794, -19259, -19719, -20172, -20620,
	-21062, -21497, -21925, -22347, -22762, -23169, -23570, -23964, -24350, -24729,
	-25101, -25465, -25821, -26169, -26509, -26841, -27165, -27481, -27788, -27988,
	-28377, -28659, -28931, -29196, -29451, -29697, -29934, -30162, -30381, -30591,
	-30791, -30982, -31163, -31335, -31497, -31650, -31792, -31925, -32047, -32160,
	-32262, -32355, -32437, -32509, -32571, -32623, -32665, -32696, -32717, -32728, -32767
};

static int16_t last_beam_x = RADAR_CENTER_X;
static int16_t last_beam_y = RADAR_CENTER_Y - RADAR_MAX_R;
static int16_t last_point_x;
static int16_t last_point_y;
static bool last_point_valid;
/*
 * 每个角度最近一次"有遮挡"的距离（mm），-1 表示该角度当前无遮挡记录。
 * 遮挡线画出来后一直保留到下一次扫过同一角度才更新——真实雷达的余辉效果。
 */
static int16_t shadow_dist_mm[181];

/*
 * overlay 开了 inversion-on（不开会花屏），面板会把写入的像素值硬件取反，
 * 所以软件写屏前统一先取反补偿，COLOR_RADAR_* 宏保持"直觉色"不变：
 * 0x0000 仍然是黑底、0x07E0 仍然是绿线。
 * 小端 MCU → SPI 按大端发 RGB565，再做一次字节序交换。
 */
static inline uint16_t lcd_pixel(uint16_t color)
{
	return __builtin_bswap16((uint16_t)~color);
}

/*
 * Decide whether a pixel belongs to the static radar frame (range rings and
 * angle spokes) without keeping a framebuffer in RAM. The test is analytic:
 * ring membership comes from the squared distance to the radar center, and
 * spoke membership from the perpendicular distance/projection against the
 * seven grid lines. It is a superset of the drawn pixels, so restoring with
 * it never erases static art; it only thickens the dotted rings slightly as
 * the beam sweeps past them.
 */
static bool static_pixel_is_set(int16_t x, int16_t y)
{
	int32_t dx = (int32_t)x - RADAR_CENTER_X;
	int32_t dy = (int32_t)y - RADAR_CENTER_Y;

	/* The half-circle radar frame only occupies the upper half. */
	if (dy > 0) {
		return false;
	}

	int32_t r2 = dx * dx + dy * dy;
	static const int16_t ring_r[] = {26, 52, 78};
	for (size_t i = 0; i < ARRAY_SIZE(ring_r); i++) {
		int32_t r = ring_r[i];
		if (r2 >= (r - 1) * (r - 1) && r2 <= (r + 1) * (r + 1)) {
			return true;
		}
	}

	static const uint8_t spokes[] = {0, 30, 60, 90, 120, 150, 180};
	for (size_t i = 0; i < ARRAY_SIZE(spokes); i++) {
		uint8_t a = spokes[i];
		/* Perpendicular distance to the spoke line, in Q15 pixel units. */
		int32_t cross = dy * cos_lut_q15[a] - dx * sin_lut_q15[a];
		/* Projection along the spoke; must fall within [0, RADAR_MAX_R]. */
		int32_t dot = -(dx * cos_lut_q15[a] + dy * sin_lut_q15[a]);
		if (dot >= 0 && dot <= (int32_t)RADAR_MAX_R * 32767 &&
		    cross >= -32767 && cross <= 32767) {
			return true;
		}
	}
	return false;
}

static void restore_static_pixel(int16_t x, int16_t y)
{
	gui_radar_draw_pixel(x, y, static_pixel_is_set(x, y) ?
				     COLOR_RADAR_GRID : COLOR_RADAR_BG);
}

/* 距离(mm) → 屏幕半径(px)，目标落在量程边缘时钳到最大半径 */
static int16_t dist_to_radius(uint16_t dist_mm, uint16_t max_range_mm)
{
	if (max_range_mm == 0) {
		return 0;
	}
	int32_t r = ((int32_t)dist_mm * RADAR_MAX_R) / max_range_mm;
	if (r > RADAR_MAX_R) {
		r = RADAR_MAX_R;
	}
	if (r < 1) {
		r = 1;
	}
	return (int16_t)r;
}

/* 沿 angle 方向的径向线段 [r_from, r_to]：画固定色或按底图恢复 */
static void spoke_segment(int16_t r_from, int16_t r_to, uint8_t angle, uint16_t color)
{
	for (int16_t r = r_from; r <= r_to; r++) {
		int16_t x = RADAR_CENTER_X - (int16_t)(((int32_t)r * cos_lut_q15[angle]) >> 15);
		int16_t y = RADAR_CENTER_Y - (int16_t)(((int32_t)r * sin_lut_q15[angle]) >> 15);
		gui_radar_draw_pixel(x, y, color);
	}
}

static void spoke_segment_restore(int16_t r_from, int16_t r_to, uint8_t angle)
{
	for (int16_t r = r_from; r <= r_to; r++) {
		int16_t x = RADAR_CENTER_X - (int16_t)(((int32_t)r * cos_lut_q15[angle]) >> 15);
		int16_t y = RADAR_CENTER_Y - (int16_t)(((int32_t)r * sin_lut_q15[angle]) >> 15);
		restore_static_pixel(x, y);
	}
}

void gui_radar_draw_pixel(int16_t x, int16_t y, uint16_t color)
{
	if (disp_dev == NULL || x < 0 || x >= 160 || y < 0 || y >= 128) {
		return;
	}
	uint16_t swapped = lcd_pixel(color);
	struct display_buffer_descriptor desc = {
		.buf_size = sizeof(uint16_t),
		.width = 1,
		.height = 1,
		.pitch = 1,
	};
	(void)display_write(disp_dev, x, y, &desc, &swapped);
}

static void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
	int16_t dx = abs(x1 - x0);
	int16_t dy = -abs(y1 - y0);
	int16_t sx = (x0 < x1) ? 1 : -1;
	int16_t sy = (y0 < y1) ? 1 : -1;
	int16_t err = dx + dy;

	while (true) {
		gui_radar_draw_pixel(x0, y0, color);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		int16_t e2 = 2 * err;
		if (e2 >= dy) {
			err += dy;
			x0 += sx;
		}
		if (e2 <= dx) {
			err += dx;
			y0 += sy;
		}
	}
}

static void draw_line_restore(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
	int16_t dx = abs(x1 - x0);
	int16_t dy = -abs(y1 - y0);
	int16_t sx = (x0 < x1) ? 1 : -1;
	int16_t sy = (y0 < y1) ? 1 : -1;
	int16_t err = dx + dy;

	while (true) {
		restore_static_pixel(x0, y0);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		int16_t e2 = 2 * err;
		if (e2 >= dy) {
			err += dy;
			x0 += sx;
		}
		if (e2 <= dx) {
			err += dx;
			y0 += sy;
		}
	}
}

static void draw_arc_point(int16_t r, uint16_t color)
{
	for (int deg = 0; deg <= 180; deg += 3) {
		int16_t x = RADAR_CENTER_X - (int16_t)(((int32_t)r * cos_lut_q15[deg]) >> 15);
		int16_t y = RADAR_CENTER_Y - (int16_t)(((int32_t)r * sin_lut_q15[deg]) >> 15);
		gui_radar_draw_pixel(x, y, color);
	}
}

int gui_radar_init(const struct device *display)
{
	disp_dev = display;
	for (size_t i = 0; i < ARRAY_SIZE(shadow_dist_mm); i++) {
		shadow_dist_mm[i] = -1;
	}
	return 0;
}

/*
 * 量程切换后的画面复位：清掉所有持久遮挡线、擦屏并重画静态底图。
 * 只能在 main 线程调用（SPI 显示事务不能跨线程交错）。
 */
void gui_radar_reset_view(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(shadow_dist_mm); i++) {
		shadow_dist_mm[i] = -1;
	}
	last_beam_x = RADAR_CENTER_X;
	last_beam_y = RADAR_CENTER_Y - RADAR_MAX_R;
	last_point_valid = false;
	gui_radar_clear_screen();
	gui_radar_draw_static_frame();
}

/* 整屏清为背景色：一行 160 像素的行缓冲逐行刷 128 次（不占用整屏显存） */
void gui_radar_clear_screen(void)
{
	if (disp_dev == NULL) {
		return;
	}

	static uint16_t row_buf[160];
	const uint16_t bg = lcd_pixel(COLOR_RADAR_BG);
	struct display_buffer_descriptor desc = {
		.buf_size = sizeof(row_buf),
		.width = 160,
		.height = 1,
		.pitch = 160,
	};

	for (int i = 0; i < 160; i++) {
		row_buf[i] = bg;
	}
	for (uint8_t y = 0; y < 128; y++) {
		(void)display_write(disp_dev, 0, y, &desc, row_buf);
	}
}

void gui_radar_draw_static_frame(void)
{
	draw_arc_point(26, COLOR_RADAR_GRID);
	draw_arc_point(52, COLOR_RADAR_GRID);
	draw_arc_point(78, COLOR_RADAR_GRID);

	const uint8_t angles[] = {0, 30, 60, 90, 120, 150, 180};
	for (int i = 0; i < ARRAY_SIZE(angles); i++) {
		uint8_t a = angles[i];
		int16_t x_end = RADAR_CENTER_X - (int16_t)(((int32_t)RADAR_MAX_R * cos_lut_q15[a]) >> 15);
		int16_t y_end = RADAR_CENTER_Y - (int16_t)(((int32_t)RADAR_MAX_R * sin_lut_q15[a]) >> 15);
		draw_line(RADAR_CENTER_X, RADAR_CENTER_Y, x_end, y_end, COLOR_RADAR_GRID);
	}
	gui_radar_draw_pixel(RADAR_CENTER_X, RADAR_CENTER_Y, COLOR_RADAR_BEAM);
}

void gui_radar_update_scan_line(uint8_t angle, uint16_t dist_mm, uint16_t max_range_mm)
{
	if (angle > 180) {
		angle = 180;
	}

	/* 1. 擦掉上一帧移动扫描线和上一个目标点 */
	draw_line_restore(RADAR_CENTER_X, RADAR_CENTER_Y, last_beam_x, last_beam_y);
	if (last_point_valid) {
		restore_static_pixel(last_point_x, last_point_y);
		restore_static_pixel(last_point_x + 1, last_point_y);
		restore_static_pixel(last_point_x, last_point_y + 1);
		restore_static_pixel(last_point_x + 1, last_point_y + 1);
	}

	/*
	 * 2. 本角度上次的遮挡线：扫到该角度就先擦掉旧线——
	 *    无论本次有没有回波，扫过即更新（无回波 = 该方向恢复畅通）。
	 */
	if (shadow_dist_mm[angle] > 0 && max_range_mm > 0) {
		spoke_segment_restore(dist_to_radius((uint16_t)shadow_dist_mm[angle], max_range_mm),
				      RADAR_MAX_R, angle);
	}
	shadow_dist_mm[angle] = -1;

	/* 3. 画新的移动扫描线（整条，绿） */
	int16_t cur_beam_x = RADAR_CENTER_X - (int16_t)(((int32_t)RADAR_MAX_R * cos_lut_q15[angle]) >> 15);
	int16_t cur_beam_y = RADAR_CENTER_Y - (int16_t)(((int32_t)RADAR_MAX_R * sin_lut_q15[angle]) >> 15);

	draw_line(RADAR_CENTER_X, RADAR_CENTER_Y, cur_beam_x, cur_beam_y, COLOR_RADAR_BEAM);
	last_beam_x = cur_beam_x;
	last_beam_y = cur_beam_y;
	last_point_valid = false;

	/* 4. 有有效回波：从物体位置到量程边缘画红色遮挡线，保留到下次扫过该角度 */
	if (dist_mm > 0 && dist_mm <= max_range_mm && max_range_mm > 0) {
		int16_t r_obj = dist_to_radius(dist_mm, max_range_mm);
		spoke_segment(r_obj, RADAR_MAX_R, angle, COLOR_RADAR_POINT);
		shadow_dist_mm[angle] = (int16_t)dist_mm;

		last_point_x = RADAR_CENTER_X - (int16_t)(((int32_t)r_obj * cos_lut_q15[angle]) >> 15);
		last_point_y = RADAR_CENTER_Y - (int16_t)(((int32_t)r_obj * sin_lut_q15[angle]) >> 15);
		gui_radar_draw_pixel(last_point_x, last_point_y, COLOR_RADAR_POINT);
		gui_radar_draw_pixel(last_point_x + 1, last_point_y, COLOR_RADAR_POINT);
		gui_radar_draw_pixel(last_point_x, last_point_y + 1, COLOR_RADAR_POINT);
		gui_radar_draw_pixel(last_point_x + 1, last_point_y + 1, COLOR_RADAR_POINT);
		last_point_valid = true;
	}
}

void gui_radar_draw_object_box(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
	draw_line(x1, y1, x2, y2, color);
}
