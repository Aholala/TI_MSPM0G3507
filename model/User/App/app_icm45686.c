/**
 * @file app_icm45686.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "app_icm45686.h"

#include "main.h"

static icm45686_module_t s_icm45686;
static imu_attitude_estimator_t s_attitude;
static imu_solution_t s_solution;
static app_icm45686_snapshot_t s_snapshot;
app_icm45686_snapshot_t g_debug_icm45686_snapshot;
imu_attitude_estimator_t g_debug_attitude_estimator;

static uint32_t enter_snapshot_lock(void)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();

	return primask;
}

static void leave_snapshot_lock(uint32_t primask)
{
	if (primask == 0u)
		__enable_irq();
}

static void update_snapshot(int process_status)
{
	uint32_t primask = enter_snapshot_lock();

	s_snapshot.solution = s_solution;
	s_snapshot.initialized = s_icm45686.initialized;
	s_snapshot.data_ready = icm45686_module_is_data_ready(&s_icm45686);
	s_snapshot.gyro_calibrated = imu_attitude_is_gyro_calibrated(&s_attitude);
	s_snapshot.gyro_calibration_count = s_attitude.gyro_calibration_count;
	s_snapshot.last_process_status = process_status;
	s_snapshot.continuous_yaw_deg = imu_attitude_get_continuous_yaw_deg(&s_attitude);
	if (process_status == 0)
		++s_snapshot.update_count;
	g_debug_icm45686_snapshot = s_snapshot;
	g_debug_attitude_estimator = s_attitude;

	leave_snapshot_lock(primask);
}

int icm45686_app_init(const icm45686_config_t *config)
{
	return icm45686_app_init_ex(config, 0);
}

int icm45686_app_init_ex(const icm45686_config_t *imu_config,
                         const imu_attitude_config_t *attitude_config)
{
	int rc;

	if (attitude_config)
		imu_attitude_init_with_config(&s_attitude, attitude_config);
	else
		imu_attitude_init(&s_attitude);

	rc = icm45686_module_init(&s_icm45686, &icm45686_bsp_default, imu_config);
	s_snapshot.last_init_status = rc;
	update_snapshot(rc);

	return rc;
}

void icm45686_app_int1_irq_handler(void)
{
	icm45686_module_int1_irq_handler(&s_icm45686);
}

uint8_t icm45686_app_is_data_ready(void)
{
	return icm45686_module_is_data_ready(&s_icm45686);
}

int icm45686_app_read_data(icm45686_data_t *data)
{
	return icm45686_module_read_data(&s_icm45686, data);
}

int icm45686_app_read_data_if_ready(icm45686_data_t *data)
{
	return icm45686_module_read_data_if_ready(&s_icm45686, data);
}

int icm45686_app_process(float dt_s, imu_solution_t *solution)
{
	int rc;
	icm45686_data_t data;
	imu_sensor_sample_t sample;

	/* Consume only completed DMA frames. Starting the next transfer never waits
	 * for DMA completion; the ISR publishes it for a later task iteration. */
	rc = icm45686_module_take_async_data(&s_icm45686, &data);
	if (rc) {
		if (icm45686_module_is_data_ready(&s_icm45686) && !s_icm45686.dma_busy)
			(void)icm45686_module_start_read_data_async(&s_icm45686);
		update_snapshot(rc);
		return rc;
	}

	sample.accel_mps2.x = data.accel_mg[0] * 0.00980665f;
	sample.accel_mps2.y = data.accel_mg[1] * 0.00980665f;
	sample.accel_mps2.z = data.accel_mg[2] * 0.00980665f;
	sample.gyro_radps.x = data.gyro_dps[0] * 0.01745329252f;
	sample.gyro_radps.y = data.gyro_dps[1] * 0.01745329252f;
	sample.gyro_radps.z = data.gyro_dps[2] * 0.01745329252f;
	sample.temp_degc = data.temp_degc;
	sample.timestamp_us = 0;

	imu_attitude_update(&s_attitude, &sample, dt_s, &s_solution);
	if (icm45686_module_is_data_ready(&s_icm45686) && !s_icm45686.dma_busy)
		(void)icm45686_module_start_read_data_async(&s_icm45686);
	if (solution)
		*solution = s_solution;

	update_snapshot(0);

	return 0;
}

const imu_solution_t *icm45686_app_get_solution(void)
{
	return &s_solution;
}

icm45686_module_t *icm45686_app_get_module(void)
{
	return &s_icm45686;
}

app_icm45686_snapshot_t icm45686_app_get_snapshot(void)
{
	app_icm45686_snapshot_t snapshot;
	uint32_t primask = enter_snapshot_lock();

	snapshot = s_snapshot;
	leave_snapshot_lock(primask);

	return snapshot;
}

float icm45686_app_get_yaw_deg(void)
{
	float yaw_deg;
	uint32_t primask = enter_snapshot_lock();

	yaw_deg = s_snapshot.solution.euler.yaw_deg;
	leave_snapshot_lock(primask);

	return yaw_deg;
}

float icm45686_app_get_continuous_yaw_deg(void)
{
	float yaw_deg;
	uint32_t primask = enter_snapshot_lock();

	yaw_deg = imu_attitude_get_continuous_yaw_deg(&s_attitude);
	leave_snapshot_lock(primask);

	return yaw_deg;
}

uint8_t icm45686_app_is_gyro_calibrated(void)
{
	return imu_attitude_is_gyro_calibrated(&s_attitude);
}

void icm45686_app_reset_yaw(void)
{
	uint32_t primask = enter_snapshot_lock();

	imu_attitude_reset_yaw(&s_attitude);
	s_solution.euler.yaw_deg = 0.0f;
	s_solution.quat = imu_quat_from_euler_rad(s_solution.euler.roll_deg * 0.01745329252f,
	                                         s_solution.euler.pitch_deg * 0.01745329252f,
	                                         0.0f);
	update_snapshot(0);
	leave_snapshot_lock(primask);
}

float icm45686_app_get_yaw_error_deg(float target_yaw_deg)
{
	float current_yaw_deg;
	uint32_t primask = enter_snapshot_lock();

	current_yaw_deg = s_snapshot.solution.euler.yaw_deg;
	leave_snapshot_lock(primask);

	return imu_angle_error_deg(target_yaw_deg, current_yaw_deg);
}
