#ifndef __LIB_ATTITUDE_H
#define __LIB_ATTITUDE_H

#include "lib_imu_types.h"
#include "lib_kalman.h"

typedef struct {
	float gravity_mps2;
	float accel_trust_min;
	float accel_trust_max;
	float accel_norm_tolerance_mps2;
	float stationary_gyro_threshold_radps;
	float stationary_accel_tolerance_mps2;
	float gyro_bias_alpha;
	float yaw_gyro_deadband_radps;
	float angle_r_min;
	float angle_r_max;
} imu_attitude_config_t;

typedef struct {
	kalman_angle_t roll_filter;
	kalman_angle_t pitch_filter;
	imu_sensor_kalman_t sensor_filter;
	imu_attitude_config_t config;
	imu_vec3_t gyro_bias_radps;
	float yaw_rad;
	uint8_t initialized;
} imu_attitude_estimator_t;

void imu_attitude_init(imu_attitude_estimator_t *estimator);
void imu_attitude_get_default_config(imu_attitude_config_t *config);
void imu_attitude_init_with_config(imu_attitude_estimator_t *estimator,
                                   const imu_attitude_config_t *config);
void imu_attitude_update(imu_attitude_estimator_t *estimator, const imu_sensor_sample_t *raw,
                         float dt_s, imu_solution_t *solution);
imu_quat_t imu_quat_from_euler_rad(float roll_rad, float pitch_rad, float yaw_rad);
imu_euler_t imu_euler_from_quat(const imu_quat_t *quat);

#endif
