/*
 * Copyright (c) 2026 Tomimi
 * SPDX-License-Identifier: Apache-2.0
 *
 * FRDM-MCXA153 ultrasonic radar integration:
 * servo scan, HC-SR04 sampling, radar rendering/object detection, and SW3 control.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "gui_radar.h"
#include "algo_object_detect.h"

LOG_MODULE_REGISTER(final_radar_system, LOG_LEVEL_INF);

static const struct gpio_dt_spec trig_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sonar_trig), gpios);
static const struct gpio_dt_spec echo_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sonar_echo), gpios);
static const struct pwm_dt_spec servo_spec = PWM_DT_SPEC_GET(DT_ALIAS(servo_pwm));
/* The application overlay maps sw0 to the board's SW3 node. */
static const struct gpio_dt_spec btn0_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
/* SW2 (P3_29, active low): cycle the radar range at runtime. */
static const struct gpio_dt_spec btn_range_spec = GPIO_DT_SPEC_GET(DT_ALIAS(range_btn), gpios);
static const struct device *const temp_dev = DEVICE_DT_GET(DT_ALIAS(ambient_temp0));

#define SERVO_MIN_PULSE_US  500
#define SERVO_MAX_PULSE_US  2500
#define STACK_SIZE          2048
#define SONAR_TIMEOUT_MS    35
#define BUTTON_DEBOUNCE_MS  50
/*
 * 舵机物理安装方向与屏上雷达角度方向相反时打开（1 = 屏幕显示 180°-实际角度）。
 * 换安装方向/换舵机后方向不对，改这一个宏即可。
 */
#define SERVO_ANGLE_REVERSED 1

struct radar_sample_t {
	uint8_t angle;
	int16_t dist_mm;
	uint16_t frame_id;
	bool is_frame_end;
};

K_MSGQ_DEFINE(radar_msgq, sizeof(struct radar_sample_t), 32, 4);
static struct k_sem sem_servo_ready;
static struct k_sem sem_echo_done;
static struct k_work_delayable button_work;
static struct k_work_delayable range_button_work;

/*
 * SW2 量程档位表（mm）：30cm 起步，逐档翻倍到 2m。
 * 小量程时同样 78px 半径分给更短距离，近距离分辨率更高。
 */
static const uint16_t range_table_mm[] = {300, 600, 1000, 2000};
#define RANGE_GEAR_COUNT ARRAY_SIZE(range_table_mm)

static volatile bool scan_paused;
static volatile uint16_t current_max_range_mm = 1000;
static volatile uint8_t g_range_idx = 2; /* 默认 1m 档 */
/* 量程切换请求标志：SW2 的 work 里置位，由 main 线程执行画面复位（SPI 事务不跨线程） */
static volatile bool range_switch_pending;
static volatile uint8_t g_current_angle;
static volatile int16_t g_latest_dist_mm = -1;
static volatile uint16_t g_current_frame_id;

#define STATUS_LOG_INTERVAL_MS 1000 /* 距离/角度状态打印周期 */
#define TEMP_LOG_INTERVAL_MS   5000 /* 温度打印与刷新周期 */

static volatile uint32_t t_rise;
static volatile uint32_t t_fall;
static volatile bool echo_flag;
static volatile bool echo_armed;
static struct gpio_callback echo_cb_data;
static struct gpio_callback btn_cb_data;
static struct gpio_callback btn_range_cb_data;

static int servo_set_angle(uint8_t angle)
{
	if (angle > 180) {
		angle = 180;
	}

	uint32_t pulse_us = SERVO_MIN_PULSE_US +
		((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180U;
	return pwm_set_pulse_dt(&servo_spec, PWM_USEC(pulse_us));
}

static void echo_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if (!echo_armed) {
		return;
	}

	int val = gpio_pin_get_dt(&echo_spec);
	if (val < 0) {
		return;
	}
	if (val > 0) {
		t_rise = k_cycle_get_32();
	} else if (echo_flag == false) {
		t_fall = k_cycle_get_32();
		echo_flag = true;
		echo_armed = false;
		k_sem_give(&sem_echo_done);
	}
}

static void button_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	scan_paused = !scan_paused;
	LOG_INF("SW3: scanning %s", scan_paused ? "paused" : "running");
}

static void button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	/* Keep the GPIO ISR short; the delayed work provides debounce and logging. */
	(void)k_work_reschedule(&button_work, K_MSEC(BUTTON_DEBOUNCE_MS));
}

/* SW2：只切档位置标志，画面复位交给 main 线程（保证 SPI 事务单线程） */
static void range_button_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	g_range_idx = (uint8_t)((g_range_idx + 1) % RANGE_GEAR_COUNT);
	current_max_range_mm = range_table_mm[g_range_idx];
	range_switch_pending = true;
	LOG_INF("SW2: range switched to %u mm (%u cm)",
		current_max_range_mm, current_max_range_mm / 10U);
}

static void range_button_callback(const struct device *dev, struct gpio_callback *cb,
				  uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	(void)k_work_reschedule(&range_button_work, K_MSEC(BUTTON_DEBOUNCE_MS));
}

static int32_t sonar_sample_distance_mm(void)
{
	/* Discard any completion left by an earlier measurement. */
	while (k_sem_take(&sem_echo_done, K_NO_WAIT) == 0) {
	}

	echo_flag = false;
	echo_armed = true;

	int ret = gpio_pin_set_dt(&trig_spec, 0);
	if (ret < 0) {
		echo_armed = false;
		return -1;
	}
	k_busy_wait(2);
	ret = gpio_pin_set_dt(&trig_spec, 1);
	if (ret < 0) {
		echo_armed = false;
		return -1;
	}
	k_busy_wait(12);
	ret = gpio_pin_set_dt(&trig_spec, 0);
	if (ret < 0) {
		echo_armed = false;
		return -1;
	}

	ret = k_sem_take(&sem_echo_done, K_MSEC(SONAR_TIMEOUT_MS));
	echo_armed = false;
	if (ret != 0 || !echo_flag) {
		/* Drain a race with the falling edge before the next trigger. */
		while (k_sem_take(&sem_echo_done, K_NO_WAIT) == 0) {
		}
		return -1;
	}

	uint32_t cycles = (t_fall >= t_rise) ? (t_fall - t_rise) :
		((UINT32_MAX - t_rise) + t_fall + 1U);
	uint32_t us = (uint32_t)k_cyc_to_us_floor64(cycles);
	int32_t dist_mm = (int32_t)((us * 343ULL) / 2000ULL);

	return (dist_mm >= 20 && dist_mm <= 4000) ? dist_mm : -1;
}

static void put_latest_sample(const struct radar_sample_t *sample)
{
	if (k_msgq_put(&radar_msgq, sample, K_NO_WAIT) == 0) {
		return;
	}

	/* Keep the producer non-blocking while making room for the newest sample. */
	struct radar_sample_t discarded;
	(void)k_msgq_get(&radar_msgq, &discarded, K_NO_WAIT);
	if (k_msgq_put(&radar_msgq, sample, K_NO_WAIT) != 0) {
		LOG_WRN("Radar queue full; sample at %u deg dropped", sample->angle);
	}
}

K_THREAD_STACK_DEFINE(servo_stack, STACK_SIZE);
static struct k_thread servo_thread_data;

static void servo_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int ret = servo_set_angle(0);
	if (ret < 0) {
		LOG_ERR("Cannot set initial servo angle (%d)", ret);
	}
	k_msleep(1000);

	int8_t step_dir = 1;
	int16_t angle = 0;
	bool first_sample = true;

	while (true) {
		if (scan_paused) {
			k_msleep(20);
			continue;
		}

		/* A frame is one complete sweep from one endpoint to the other. */
		if (!first_sample && angle == 0 && step_dir < 0) {
			g_current_frame_id++;
		}
		first_sample = false;

		ret = servo_set_angle((uint8_t)angle);
		if (ret < 0) {
			LOG_ERR("Servo angle %d failed (%d)", angle, ret);
		}
		g_current_angle = (uint8_t)angle;
		k_msleep(18);
		k_sem_give(&sem_servo_ready);

		angle += step_dir;
		if (angle >= 180) {
			angle = 180;
			step_dir = -1;
			k_msleep(100);
		} else if (angle <= 0) {
			angle = 0;
			step_dir = 1;
			k_msleep(100);
		}
	}
}

K_THREAD_STACK_DEFINE(sonar_stack, STACK_SIZE);
static struct k_thread sonar_thread_data;

static void sonar_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		(void)k_sem_take(&sem_servo_ready, K_FOREVER);
		struct radar_sample_t sample = {
			.angle = SERVO_ANGLE_REVERSED ? (uint8_t)(180 - g_current_angle) :
							(uint8_t)g_current_angle,
			.dist_mm = (int16_t)sonar_sample_distance_mm(),
			.frame_id = g_current_frame_id,
			.is_frame_end = (g_current_angle == 180),
		};
		put_latest_sample(&sample);
	}
}

static bool init_gpio(void)
{
	if (!gpio_is_ready_dt(&trig_spec) || !gpio_is_ready_dt(&echo_spec)) {
		LOG_ERR("Sonar GPIO device is not ready");
		return false;
	}
	if (gpio_pin_configure_dt(&trig_spec, GPIO_OUTPUT_INACTIVE) < 0 ||
	    gpio_pin_configure_dt(&echo_spec, GPIO_INPUT) < 0) {
		LOG_ERR("Cannot configure sonar GPIO");
		return false;
	}

	gpio_init_callback(&echo_cb_data, echo_gpio_callback, BIT(echo_spec.pin));
	if (gpio_add_callback(echo_spec.port, &echo_cb_data) < 0 ||
	    gpio_pin_interrupt_configure_dt(&echo_spec, GPIO_INT_EDGE_BOTH) < 0) {
		LOG_ERR("Cannot configure echo interrupt");
		return false;
	}

	if (!gpio_is_ready_dt(&btn0_spec) || gpio_pin_configure_dt(&btn0_spec, GPIO_INPUT) < 0) {
		LOG_ERR("SW3 GPIO is not ready or cannot be configured");
		return false;
	}
	gpio_init_callback(&btn_cb_data, button_callback, BIT(btn0_spec.pin));
	if (gpio_add_callback(btn0_spec.port, &btn_cb_data) < 0 ||
	    gpio_pin_interrupt_configure_dt(&btn0_spec, GPIO_INT_EDGE_TO_ACTIVE) < 0) {
		LOG_ERR("Cannot configure SW3 interrupt");
		return false;
	}

	if (!gpio_is_ready_dt(&btn_range_spec) ||
	    gpio_pin_configure_dt(&btn_range_spec, GPIO_INPUT) < 0) {
		LOG_ERR("SW2 GPIO is not ready or cannot be configured");
		return false;
	}
	gpio_init_callback(&btn_range_cb_data, range_button_callback, BIT(btn_range_spec.pin));
	if (gpio_add_callback(btn_range_spec.port, &btn_range_cb_data) < 0 ||
	    gpio_pin_interrupt_configure_dt(&btn_range_spec, GPIO_INT_EDGE_TO_ACTIVE) < 0) {
		LOG_ERR("Cannot configure SW2 interrupt");
		return false;
	}
	return true;
}

/* 低频状态日志：1s 打印一次角度+距离（无回波时打印 no-echo），5s 追加一次温度 */
static void update_status_log(uint32_t *last_status_ms, uint32_t *last_temp_ms)
{
	uint32_t now = k_uptime_get_32();
	if ((uint32_t)(now - *last_status_ms) < STATUS_LOG_INTERVAL_MS) {
		return;
	}
	*last_status_ms = now;

	int16_t dist = g_latest_dist_mm;
	if (dist > 0) {
		LOG_INF("angle=%3u deg  dist=%4d mm", g_current_angle, dist);
	} else {
		LOG_INF("angle=%3u deg  dist=  -- (no echo)", g_current_angle);
	}

	if ((uint32_t)(now - *last_temp_ms) < TEMP_LOG_INTERVAL_MS) {
		return;
	}
	*last_temp_ms = now;

	if (!device_is_ready(temp_dev)) {
		LOG_WRN("P3T1755 device is not ready");
		return;
	}

	struct sensor_value value;
	int ret = sensor_sample_fetch_chan(temp_dev, SENSOR_CHAN_AMBIENT_TEMP);
	if (ret == 0) {
		ret = sensor_channel_get(temp_dev, SENSOR_CHAN_AMBIENT_TEMP, &value);
	}
	if (ret < 0) {
		LOG_WRN("P3T1755 read failed (%d)", ret);
		return;
	}

	double temp_c = sensor_value_to_double(&value);
	LOG_INF("Ambient temperature: %.1f C", temp_c);
}

static int16_t scan_frame_buffer[RADAR_MAX_POINTS];
static struct detected_object detected_objs[MAX_DETECTED_OBJECTS];

int main(void)
{
	const struct device *display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	int ret;

	LOG_INF("==========================================================");
	LOG_INF("   FRDM-MCXA153 Ultrasonic Radar and Reconstructor System");
	LOG_INF("==========================================================");

	if (!device_is_ready(display_dev)) {
		LOG_ERR("Display device is not ready");
		return -ENODEV;
	}
	LOG_INF("Display device ready");

	if (!pwm_is_ready_dt(&servo_spec)) {
		LOG_ERR("Servo PWM device is not ready");
		return -ENODEV;
	}
	LOG_INF("Servo PWM ready");

	if (!init_gpio()) {
		LOG_ERR("GPIO initialization failed");
		return -EIO;
	}
	LOG_INF("Sonar/echo/button GPIO ready");

	ret = display_blanking_off(display_dev);
	if (ret < 0) {
		LOG_ERR("display_blanking_off failed (%d)", ret);
		return -EIO;
	}
	ret = gui_radar_init(display_dev);
	if (ret < 0) {
		LOG_ERR("gui_radar_init failed (%d)", ret);
		return -EIO;
	}
	LOG_INF("Display blanking off, GUI ready");

	k_sem_init(&sem_servo_ready, 0, 1);
	k_sem_init(&sem_echo_done, 0, 1);
	k_work_init_delayable(&button_work, button_work_handler);
	k_work_init_delayable(&range_button_work, range_button_work_handler);
	algo_init(NULL);
	for (size_t i = 0; i < ARRAY_SIZE(scan_frame_buffer); i++) {
		scan_frame_buffer[i] = -1;
	}

	k_thread_create(&servo_thread_data, servo_stack, K_THREAD_STACK_SIZEOF(servo_stack),
			servo_thread, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
	k_thread_create(&sonar_thread_data, sonar_stack, K_THREAD_STACK_SIZEOF(sonar_stack),
			sonar_thread, NULL, NULL, NULL, 5, 0, K_NO_WAIT);

	gui_radar_clear_screen();
	gui_radar_draw_static_frame();
	LOG_INF("Screen cleared, static radar frame drawn, entering main loop");

	struct radar_sample_t sample;
	uint16_t active_frame_id = UINT16_MAX;
	uint32_t last_status_ms = 0;
	uint32_t last_temp_ms = 0;

	while (true) {
		if (range_switch_pending) {
			range_switch_pending = false;
			gui_radar_reset_view();
		}
		if (k_msgq_get(&radar_msgq, &sample, K_MSEC(100)) != 0) {
			update_status_log(&last_status_ms, &last_temp_ms);
			continue;
		}
		update_status_log(&last_status_ms, &last_temp_ms);

		if (sample.angle >= RADAR_MAX_POINTS) {
			continue;
		}
		if (sample.dist_mm > 0) {
			g_latest_dist_mm = sample.dist_mm;
		}
		if (sample.frame_id != active_frame_id) {
			for (size_t i = 0; i < ARRAY_SIZE(scan_frame_buffer); i++) {
				scan_frame_buffer[i] = -1;
			}
			active_frame_id = sample.frame_id;
		}

		/* Invalid samples are retained as -1 for the algorithm, but do not draw points. */
		gui_radar_update_scan_line(sample.angle,
					   sample.dist_mm > 0 ? (uint16_t)sample.dist_mm : 0,
					   current_max_range_mm);
		scan_frame_buffer[sample.angle] = sample.dist_mm;

		if (!sample.is_frame_end) {
			continue;
		}

		uint8_t count = algo_process_scan_frame(scan_frame_buffer, detected_objs,
							MAX_DETECTED_OBJECTS);
		if (count > 0) {
			struct detected_object *best = &detected_objs[0];
			LOG_INF("object #%u: %u-%u deg  L=%.1fcm  D=%ucm",
				best->id, best->start_angle, best->end_angle,
				(double)best->calculated_length_cm, best->min_dist_mm / 10);
		} else {
			LOG_INF("frame %u: no object", (unsigned int)sample.frame_id);
		}
	}
}
