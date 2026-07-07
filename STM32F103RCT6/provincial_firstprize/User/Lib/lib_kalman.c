#include "lib_kalman.h"

#include <string.h>

void kalman_1d_init(kalman_1d_t *kf, float q, float r, float initial)
{
	if (!kf)
		return;

	kf->q = q;
	kf->r = r;
	kf->x = initial;
	kf->p = 1.0f;
	kf->initialized = 1;
}

float kalman_1d_update(kalman_1d_t *kf, float measurement)
{
	float k;

	if (!kf)
		return measurement;

	if (!kf->initialized)
		kalman_1d_init(kf, 0.01f, 1.0f, measurement);

	kf->p += kf->q;
	k = kf->p / (kf->p + kf->r);
	kf->x += k * (measurement - kf->x);
	kf->p *= 1.0f - k;

	return kf->x;
}

void kalman_angle_init(kalman_angle_t *kf, float q_angle, float q_bias, float r_measure,
                       float initial_angle)
{
	if (!kf)
		return;

	memset(kf, 0, sizeof(*kf));
	kf->q_angle = q_angle;
	kf->q_bias = q_bias;
	kf->r_measure = r_measure;
	kf->angle = initial_angle;
	kf->initialized = 1;
}

float kalman_angle_update(kalman_angle_t *kf, float measured_angle, float measured_rate,
                          float dt_s)
{
	float s;
	float k0;
	float k1;
	float y;
	float p00_temp;
	float p01_temp;

	if (!kf)
		return measured_angle;

	if (!kf->initialized)
		kalman_angle_init(kf, 0.001f, 0.003f, 0.03f, measured_angle);

	if (dt_s <= 0.0f)
		return kf->angle;

	kf->rate = measured_rate - kf->bias;
	kf->angle += dt_s * kf->rate;

	kf->p[0][0] += dt_s * (dt_s * kf->p[1][1] - kf->p[0][1] - kf->p[1][0] + kf->q_angle);
	kf->p[0][1] -= dt_s * kf->p[1][1];
	kf->p[1][0] -= dt_s * kf->p[1][1];
	kf->p[1][1] += kf->q_bias * dt_s;

	s = kf->p[0][0] + kf->r_measure;
	k0 = kf->p[0][0] / s;
	k1 = kf->p[1][0] / s;
	y = measured_angle - kf->angle;

	kf->angle += k0 * y;
	kf->bias += k1 * y;

	p00_temp = kf->p[0][0];
	p01_temp = kf->p[0][1];

	kf->p[0][0] -= k0 * p00_temp;
	kf->p[0][1] -= k0 * p01_temp;
	kf->p[1][0] -= k1 * p00_temp;
	kf->p[1][1] -= k1 * p01_temp;

	return kf->angle;
}

void imu_sensor_kalman_init(imu_sensor_kalman_t *filter)
{
	if (!filter)
		return;

	memset(filter, 0, sizeof(*filter));

	for (int i = 0; i < 3; i++) {
		kalman_1d_init(&filter->accel[i], 0.05f, 4.0f, 0.0f);
		kalman_1d_init(&filter->gyro[i], 0.01f, 0.5f, 0.0f);
	}
}

void imu_sensor_kalman_update(imu_sensor_kalman_t *filter, const imu_sensor_sample_t *raw,
                              imu_sensor_sample_t *filtered)
{
	if (!filter || !raw || !filtered)
		return;

	*filtered = *raw;
	filtered->accel_mps2.x = kalman_1d_update(&filter->accel[0], raw->accel_mps2.x);
	filtered->accel_mps2.y = kalman_1d_update(&filter->accel[1], raw->accel_mps2.y);
	filtered->accel_mps2.z = kalman_1d_update(&filter->accel[2], raw->accel_mps2.z);
	filtered->gyro_radps.x = kalman_1d_update(&filter->gyro[0], raw->gyro_radps.x);
	filtered->gyro_radps.y = kalman_1d_update(&filter->gyro[1], raw->gyro_radps.y);
	filtered->gyro_radps.z = kalman_1d_update(&filter->gyro[2], raw->gyro_radps.z);
}
