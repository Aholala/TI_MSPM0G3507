#ifndef __LIB_IMU_TYPES_H
#define __LIB_IMU_TYPES_H

#include <stdint.h>

typedef struct {
	float x;
	float y;
	float z;
} imu_vec3_t;

typedef struct {
	imu_vec3_t accel_mps2;
	imu_vec3_t gyro_radps;
	float temp_degc;
	uint32_t timestamp_us;
} imu_sensor_sample_t;

typedef struct {
	float w;
	float x;
	float y;
	float z;
} imu_quat_t;

typedef struct {
	float roll_deg;
	float pitch_deg;
	float yaw_deg;
} imu_euler_t;

typedef struct {
	imu_vec3_t gyro_bias_radps;
	float accel_trust;
	uint8_t stationary;
} imu_compensation_t;

typedef struct {
	imu_sensor_sample_t raw;
	imu_sensor_sample_t filtered;
	imu_quat_t quat;
	imu_euler_t euler;
	imu_compensation_t compensation;
} imu_solution_t;

#endif
