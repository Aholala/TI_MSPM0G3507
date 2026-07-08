#ifndef __LIB_KALMAN_H
#define __LIB_KALMAN_H

#include "lib_imu_types.h"

typedef struct {
	float q;
	float r;
	float x;
	float p;
	uint8_t initialized;
} kalman_1d_t;

typedef struct {
	float q_angle;
	float q_bias;
	float r_measure;
	float angle;
	float bias;
	float rate;
	float p[2][2];
	uint8_t initialized;
} kalman_angle_t;

typedef struct {
	kalman_1d_t accel[3];
	kalman_1d_t gyro[3];
} imu_sensor_kalman_t;

void kalman_1d_init(kalman_1d_t *kf, float q, float r, float initial);
float kalman_1d_update(kalman_1d_t *kf, float measurement);

void kalman_angle_init(kalman_angle_t *kf, float q_angle, float q_bias, float r_measure,
                       float initial_angle);
float kalman_angle_update(kalman_angle_t *kf, float measured_angle, float measured_rate,
                          float dt_s);

void imu_sensor_kalman_init(imu_sensor_kalman_t *filter);
void imu_sensor_kalman_update(imu_sensor_kalman_t *filter, const imu_sensor_sample_t *raw,
                              imu_sensor_sample_t *filtered);

#endif
