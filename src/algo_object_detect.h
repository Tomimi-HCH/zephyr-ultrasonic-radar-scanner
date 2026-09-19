/*
 * Copyright (c) 2026 Tomimi
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ALGO_OBJECT_DETECT_H
#define __ALGO_OBJECT_DETECT_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#define RADAR_MAX_POINTS 181
#define MAX_DETECTED_OBJECTS 3

struct detected_object {
	uint8_t id;
	uint8_t start_angle;
	uint8_t end_angle;
	uint16_t start_dist_mm;
	uint16_t end_dist_mm;
	uint16_t min_dist_mm;
	uint8_t center_angle;
	float calculated_length_cm;
};

struct algo_config {
	uint16_t dist_continuity_threshold_mm;
	uint8_t min_cluster_points;
	float beam_compensation_deg;
};

void algo_init(const struct algo_config *cfg);
uint8_t algo_process_scan_frame(const int16_t *scan_frame_mm, struct detected_object *objects_out, uint8_t max_objs);
float algo_calculate_width_cosine(uint16_t r1_mm, uint8_t th1_deg, uint16_t r2_mm, uint8_t th2_deg, float beam_comp);

#endif /* __ALGO_OBJECT_DETECT_H */
