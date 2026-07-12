#ifndef LIB_ADAPTIVE_MAHONY_H
#define LIB_ADAPTIVE_MAHONY_H

#include <stdint.h>

#include "lib_imu_types.h"

#define ADAPTIVE_MAHONY_INNOVATION_WINDOW 10U

typedef struct {
	float x;
	float p;
	float q;
	float r;
	float innovation[ADAPTIVE_MAHONY_INNOVATION_WINDOW];
	uint8_t index;
	uint8_t full;
	uint8_t initialized;
} adaptive_gyro_kalman_t;

typedef struct {
	float kp_static;
	float kp_dynamic;
	float ki_static;
	float integral_limit_radps;
	float gyro_q_initial;
	float gyro_q_min;
	float gyro_q_max;
	float gyro_r;
} adaptive_mahony_config_t;

typedef struct {
	imu_quat_t quat;
	imu_vec3_t integral_error;
	imu_vec3_t filtered_gyro_radps;
	adaptive_gyro_kalman_t gyro_filter[3];
	adaptive_mahony_config_t config;
	uint8_t initialized;
} adaptive_mahony_t;

void adaptive_mahony_get_default_config(adaptive_mahony_config_t *config);
void adaptive_mahony_init(adaptive_mahony_t *filter,
	                      const adaptive_mahony_config_t *config);
void adaptive_mahony_reset_heading(adaptive_mahony_t *filter);
void adaptive_mahony_update(adaptive_mahony_t *filter, imu_vec3_t accel_mps2,
	                         imu_vec3_t gyro_radps, float accel_trust,
	                         uint8_t stationary, float dt_s);

#endif
