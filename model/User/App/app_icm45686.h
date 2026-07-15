/**
 * @file app_icm45686.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __ICM45686_APP_H
#define __ICM45686_APP_H

#include <stdint.h>

#include "../lib/lib_attitude.h"
#include "../module/module_icm45686.h"

#define APP_ICM45686_PERIOD_MS 5U

typedef struct {
	imu_solution_t solution;
	uint8_t initialized;
	uint8_t data_ready;
	uint8_t gyro_calibrated;
	uint16_t gyro_calibration_count;
	int last_init_status;
	int last_process_status;
	uint32_t update_count;
	float continuous_yaw_deg;
} app_icm45686_snapshot_t;

extern app_icm45686_snapshot_t g_debug_icm45686_snapshot;
extern imu_attitude_estimator_t g_debug_attitude_estimator;

int icm45686_app_init(const icm45686_config_t *config);
int icm45686_app_init_ex(const icm45686_config_t *imu_config,
                         const imu_attitude_config_t *attitude_config);
void icm45686_app_int1_irq_handler(void);
uint8_t icm45686_app_is_data_ready(void);
int icm45686_app_read_data(icm45686_data_t *data);
int icm45686_app_read_data_if_ready(icm45686_data_t *data);
int icm45686_app_process(float dt_s, imu_solution_t *solution);
const imu_solution_t *icm45686_app_get_solution(void);
icm45686_module_t *icm45686_app_get_module(void);
app_icm45686_snapshot_t icm45686_app_get_snapshot(void);
float icm45686_app_get_yaw_deg(void);
float icm45686_app_get_continuous_yaw_deg(void);
uint8_t icm45686_app_is_gyro_calibrated(void);
void icm45686_app_reset_yaw(void);
float icm45686_app_get_yaw_error_deg(float target_yaw_deg);

#endif
