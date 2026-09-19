/*
 * Copyright (c) 2026 Tomimi
 * SPDX-License-Identifier: Apache-2.0
 */

#include "algo_object_detect.h"
#include <math.h>
#include <stdlib.h>

#define PI_F 3.14159265358979323846f

static struct algo_config g_cfg = {
	.dist_continuity_threshold_mm = 60,
	.min_cluster_points = 3,
	.beam_compensation_deg = 8.0f,
};

void algo_init(const struct algo_config *cfg)
{
	if (cfg != NULL) {
		g_cfg = *cfg;
	}
}

float algo_calculate_width_cosine(uint16_t r1_mm, uint8_t th1_deg, uint16_t r2_mm, uint8_t th2_deg, float beam_comp)
{
	if (th2_deg <= th1_deg) return 0.0f;

	float delta_deg = (float)(th2_deg - th1_deg) - beam_comp;
	if (delta_deg < 1.0f) delta_deg = 1.0f;

	float rad = delta_deg * (PI_F / 180.0f);
	float r1_cm = (float)r1_mm / 10.0f;
	float r2_cm = (float)r2_mm / 10.0f;

	float l_sq = (r1_cm * r1_cm) + (r2_cm * r2_cm) - (2.0f * r1_cm * r2_cm * cosf(rad));
	if (l_sq < 0.0f) return 0.0f;

	return sqrtf(l_sq);
}

uint8_t algo_process_scan_frame(const int16_t *scan_frame_mm, struct detected_object *objects_out, uint8_t max_objs)
{
	uint8_t obj_count = 0;
	bool in_cluster = false;
	uint8_t cluster_start_angle = 0;
	uint16_t cluster_start_dist = 0;
	uint16_t cluster_min_dist = 0xFFFF;
	uint8_t cluster_min_angle = 0;
	int16_t last_valid_dist = -1;
	uint8_t cluster_pts = 0;

	for (uint8_t angle = 0; angle <= 180; angle++) {
		int16_t dist = scan_frame_mm[angle];
		bool is_valid = (dist >= 20 && dist <= 2000);

		if (is_valid) {
			if (!in_cluster) {
				in_cluster = true;
				cluster_start_angle = angle;
				cluster_start_dist = (uint16_t)dist;
				cluster_min_dist = (uint16_t)dist;
				cluster_min_angle = angle;
				cluster_pts = 1;
				last_valid_dist = dist;
			} else {
				int16_t diff = abs(dist - last_valid_dist);
				if (diff <= g_cfg.dist_continuity_threshold_mm) {
					cluster_pts++;
					last_valid_dist = dist;
					if ((uint16_t)dist < cluster_min_dist) {
						cluster_min_dist = (uint16_t)dist;
						cluster_min_angle = angle;
					}
				} else {
					if (cluster_pts >= g_cfg.min_cluster_points && obj_count < max_objs) {
						uint8_t end_ang = angle - 1;
						uint16_t end_dst = (uint16_t)last_valid_dist;

						objects_out[obj_count].id = obj_count + 1;
						objects_out[obj_count].start_angle = cluster_start_angle;
						objects_out[obj_count].end_angle = end_ang;
						objects_out[obj_count].start_dist_mm = cluster_start_dist;
						objects_out[obj_count].end_dist_mm = end_dst;
						objects_out[obj_count].min_dist_mm = cluster_min_dist;
						objects_out[obj_count].center_angle = cluster_min_angle;
						objects_out[obj_count].calculated_length_cm =
							algo_calculate_width_cosine(cluster_start_dist, cluster_start_angle,
										    end_dst, end_ang, g_cfg.beam_compensation_deg);
						obj_count++;
					}

					cluster_start_angle = angle;
					cluster_start_dist = (uint16_t)dist;
					cluster_min_dist = (uint16_t)dist;
					cluster_min_angle = angle;
					cluster_pts = 1;
					last_valid_dist = dist;
				}
			}
		} else {
			if (in_cluster) {
				if (cluster_pts >= g_cfg.min_cluster_points && obj_count < max_objs) {
					uint8_t end_ang = angle - 1;
					uint16_t end_dst = (uint16_t)last_valid_dist;

					objects_out[obj_count].id = obj_count + 1;
					objects_out[obj_count].start_angle = cluster_start_angle;
					objects_out[obj_count].end_angle = end_ang;
					objects_out[obj_count].start_dist_mm = cluster_start_dist;
					objects_out[obj_count].end_dist_mm = end_dst;
					objects_out[obj_count].min_dist_mm = cluster_min_dist;
					objects_out[obj_count].center_angle = cluster_min_angle;
					objects_out[obj_count].calculated_length_cm =
						algo_calculate_width_cosine(cluster_start_dist, cluster_start_angle,
									    end_dst, end_ang, g_cfg.beam_compensation_deg);
					obj_count++;
				}
				in_cluster = false;
				cluster_pts = 0;
			}
		}
	}

	if (in_cluster && cluster_pts >= g_cfg.min_cluster_points && obj_count < max_objs) {
		objects_out[obj_count].id = obj_count + 1;
		objects_out[obj_count].start_angle = cluster_start_angle;
		objects_out[obj_count].end_angle = 180;
		objects_out[obj_count].start_dist_mm = cluster_start_dist;
		objects_out[obj_count].end_dist_mm = (uint16_t)last_valid_dist;
		objects_out[obj_count].min_dist_mm = cluster_min_dist;
		objects_out[obj_count].center_angle = cluster_min_angle;
		objects_out[obj_count].calculated_length_cm =
			algo_calculate_width_cosine(cluster_start_dist, cluster_start_angle,
						    (uint16_t)last_valid_dist, 180, g_cfg.beam_compensation_deg);
		obj_count++;
	}

	return obj_count;
}
