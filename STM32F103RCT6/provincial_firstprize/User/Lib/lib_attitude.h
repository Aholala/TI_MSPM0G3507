#ifndef __LIB_ATTITUDE_H
#define __LIB_ATTITUDE_H

#include "lib_imu_types.h"
#include "lib_kalman.h"
#include "lib_adaptive_mahony.h"

typedef struct {
	float gravity_mps2;
	float accel_trust_min;
	float accel_trust_max;
	float accel_norm_tolerance_mps2;
	float stationary_gyro_threshold_radps;
	float stationary_accel_tolerance_mps2;
	float gyro_bias_alpha;
	float yaw_gyro_deadband_radps;
	float yaw_gyro_scale;
	float angle_r_min;
	float angle_r_max;
	float euler_lpf_time_constant_s;
	uint16_t gyro_calibration_samples;
} imu_attitude_config_t;

typedef struct {
	kalman_angle_t roll_filter;
	kalman_angle_t pitch_filter;
	imu_sensor_kalman_t sensor_filter;
	adaptive_mahony_t fusion;
	imu_attitude_config_t config;
	imu_vec3_t gyro_bias_radps;
	imu_vec3_t gyro_calibration_sum;
	imu_vec3_t gyro_calibration_sum_sq;
	float yaw_rad;
	float continuous_yaw_rad;
	float previous_yaw_rate_radps;
	float previous_wrapped_yaw_deg;
	imu_euler_t filtered_euler;
	uint16_t gyro_calibration_count;
	uint8_t gyro_calibrated;
	uint8_t euler_lpf_initialized;
	uint8_t yaw_unwrap_initialized;
	uint8_t initialized;
} imu_attitude_estimator_t;

void imu_attitude_init(imu_attitude_estimator_t *estimator);
void imu_attitude_get_default_config(imu_attitude_config_t *config);
void imu_attitude_init_with_config(imu_attitude_estimator_t *estimator,
                                   const imu_attitude_config_t *config);
void imu_attitude_update(imu_attitude_estimator_t *estimator, const imu_sensor_sample_t *raw,
                         float dt_s, imu_solution_t *solution);
void imu_attitude_reset_yaw(imu_attitude_estimator_t *estimator);
uint8_t imu_attitude_is_gyro_calibrated(const imu_attitude_estimator_t *estimator);
float imu_attitude_get_continuous_yaw_deg(const imu_attitude_estimator_t *estimator);
float imu_angle_wrap_deg(float angle_deg);
float imu_angle_error_deg(float target_deg, float current_deg);
imu_quat_t imu_quat_from_euler_rad(float roll_rad, float pitch_rad, float yaw_rad);
imu_euler_t imu_euler_from_quat(const imu_quat_t *quat);

#endif
