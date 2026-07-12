#include "lib_adaptive_mahony.h"

#include <math.h>
#include <string.h>

static float clampf_local(float value, float min_value, float max_value)
{
	if (value < min_value)
		return min_value;
	if (value > max_value)
		return max_value;
	return value;
}

static void normalize_quat(imu_quat_t *q)
{
	float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);

	if (norm <= 1.0e-8f) {
		q->w = 1.0f;
		q->x = q->y = q->z = 0.0f;
		return;
	}

	q->w /= norm;
	q->x /= norm;
	q->y /= norm;
	q->z /= norm;
}

static void seed_from_accel(adaptive_mahony_t *filter, imu_vec3_t accel)
{
	float norm = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
	float roll;
	float pitch;
	float cr, sr, cp, sp;

	if (norm <= 1.0e-6f)
		return;

	accel.x /= norm;
	accel.y /= norm;
	accel.z /= norm;
	roll = atan2f(accel.y, accel.z);
	pitch = atan2f(-accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z));
	cr = cosf(roll * 0.5f);
	sr = sinf(roll * 0.5f);
	cp = cosf(pitch * 0.5f);
	sp = sinf(pitch * 0.5f);
	filter->quat.w = cr * cp;
	filter->quat.x = sr * cp;
	filter->quat.y = cr * sp;
	filter->quat.z = -sr * sp;
	normalize_quat(&filter->quat);
	filter->initialized = 1U;
}

static float adaptive_gyro_update(adaptive_gyro_kalman_t *kf, float measurement,
	                              const adaptive_mahony_config_t *config)
{
	float predicted_p;
	float innovation;
	float gain;

	if (!kf->initialized) {
		kf->x = measurement;
		kf->p = 1.0f;
		if (kf->q <= 0.0f)
			kf->q = config->gyro_q_initial;
		if (kf->r <= 0.0f)
			kf->r = config->gyro_r;
		kf->initialized = 1U;
		return measurement;
	}

	predicted_p = kf->p + kf->q;
	innovation = measurement - kf->x;
	gain = predicted_p / (predicted_p + kf->r);
	kf->x += gain * innovation;
	kf->p = (1.0f - gain) * predicted_p;

	kf->innovation[kf->index++] = innovation;
	if (kf->index >= ADAPTIVE_MAHONY_INNOVATION_WINDOW) {
		kf->index = 0U;
		kf->full = 1U;
	}

	if (kf->full) {
		float energy = 0.0f;
		for (uint8_t i = 0U; i < ADAPTIVE_MAHONY_INNOVATION_WINDOW; ++i)
			energy += kf->innovation[i] * kf->innovation[i];
		energy /= (float)ADAPTIVE_MAHONY_INNOVATION_WINDOW;
		kf->q = clampf_local(gain * gain * energy, config->gyro_q_min,
		                    config->gyro_q_max);
	}

	return kf->x;
}

void adaptive_mahony_get_default_config(adaptive_mahony_config_t *config)
{
	if (!config)
		return;

	config->kp_static = 4.0f;
	config->kp_dynamic = 0.02f;
	config->ki_static = 0.002f;
	config->integral_limit_radps = 0.05f;
	config->gyro_q_initial = 1.0e-6f;
	config->gyro_q_min = 1.0e-9f;
	config->gyro_q_max = 5.0e-3f;
	config->gyro_r = 2.5e-6f;
}

void adaptive_mahony_init(adaptive_mahony_t *filter,
	                      const adaptive_mahony_config_t *config)
{
	if (!filter)
		return;

	memset(filter, 0, sizeof(*filter));
	if (config)
		filter->config = *config;
	else
		adaptive_mahony_get_default_config(&filter->config);
	filter->quat.w = 1.0f;
}

void adaptive_mahony_reset_heading(adaptive_mahony_t *filter)
{
	float roll;
	float pitch;
	float cr, sr, cp, sp;
	imu_quat_t *q;

	if (!filter)
		return;

	q = &filter->quat;
	roll = atan2f(2.0f * (q->w * q->x + q->y * q->z),
	              1.0f - 2.0f * (q->x * q->x + q->y * q->y));
	pitch = asinf(clampf_local(2.0f * (q->w * q->y - q->z * q->x), -1.0f, 1.0f));
	cr = cosf(roll * 0.5f);
	sr = sinf(roll * 0.5f);
	cp = cosf(pitch * 0.5f);
	sp = sinf(pitch * 0.5f);
	q->w = cr * cp;
	q->x = sr * cp;
	q->y = cr * sp;
	q->z = -sr * sp;
	normalize_quat(q);
}

void adaptive_mahony_update(adaptive_mahony_t *filter, imu_vec3_t accel_mps2,
	                         imu_vec3_t gyro_radps, float accel_trust,
	                         uint8_t stationary, float dt_s)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float kp;
	float ki;
	float wx, wy, wz;
	float half_wx, half_wy, half_wz;
	float k1w, k1x, k1y, k1z;
	imu_quat_t mid;
	imu_quat_t q;

	if (!filter || dt_s <= 0.0f)
		return;
	if (!filter->initialized)
		seed_from_accel(filter, accel_mps2);
	if (!filter->initialized)
		return;

	filter->filtered_gyro_radps.x =
	    adaptive_gyro_update(&filter->gyro_filter[0], gyro_radps.x, &filter->config);
	filter->filtered_gyro_radps.y =
	    adaptive_gyro_update(&filter->gyro_filter[1], gyro_radps.y, &filter->config);
	filter->filtered_gyro_radps.z =
	    adaptive_gyro_update(&filter->gyro_filter[2], gyro_radps.z, &filter->config);
	wx = filter->filtered_gyro_radps.x;
	wy = filter->filtered_gyro_radps.y;
	wz = filter->filtered_gyro_radps.z;

	norm = sqrtf(accel_mps2.x * accel_mps2.x + accel_mps2.y * accel_mps2.y +
	             accel_mps2.z * accel_mps2.z);
	if (norm <= 1.0e-6f)
		return;
	accel_mps2.x /= norm;
	accel_mps2.y /= norm;
	accel_mps2.z /= norm;
	accel_trust = clampf_local(accel_trust, 0.0f, 1.0f);

	q = filter->quat;
	vx = q.x * q.z - q.w * q.y;
	vy = q.w * q.x + q.y * q.z;
	vz = q.w * q.w - 0.5f + q.z * q.z;
	ex = accel_mps2.y * vz - accel_mps2.z * vy;
	ey = accel_mps2.z * vx - accel_mps2.x * vz;
	ez = accel_mps2.x * vy - accel_mps2.y * vx;

	kp = (stationary ? filter->config.kp_static : filter->config.kp_dynamic) * accel_trust;
	ki = stationary ? filter->config.ki_static : 0.0f;
	if (ki > 0.0f) {
		filter->integral_error.x = clampf_local(filter->integral_error.x + ki * ex * dt_s,
		    -filter->config.integral_limit_radps, filter->config.integral_limit_radps);
		filter->integral_error.y = clampf_local(filter->integral_error.y + ki * ey * dt_s,
		    -filter->config.integral_limit_radps, filter->config.integral_limit_radps);
		filter->integral_error.z = clampf_local(filter->integral_error.z + ki * ez * dt_s,
		    -filter->config.integral_limit_radps, filter->config.integral_limit_radps);
	} else {
		filter->integral_error.x = 0.0f;
		filter->integral_error.y = 0.0f;
		filter->integral_error.z = 0.0f;
	}
	wx += kp * ex + filter->integral_error.x;
	wy += kp * ey + filter->integral_error.y;
	wz += kp * ez + filter->integral_error.z;

	half_wx = 0.5f * wx;
	half_wy = 0.5f * wy;
	half_wz = 0.5f * wz;
	k1w = -q.x * half_wx - q.y * half_wy - q.z * half_wz;
	k1x = q.w * half_wx + q.y * half_wz - q.z * half_wy;
	k1y = q.w * half_wy - q.x * half_wz + q.z * half_wx;
	k1z = q.w * half_wz + q.x * half_wy - q.y * half_wx;
	mid.w = q.w + k1w * dt_s * 0.5f;
	mid.x = q.x + k1x * dt_s * 0.5f;
	mid.y = q.y + k1y * dt_s * 0.5f;
	mid.z = q.z + k1z * dt_s * 0.5f;
	normalize_quat(&mid);
	filter->quat.w = q.w + (-mid.x * half_wx - mid.y * half_wy - mid.z * half_wz) * dt_s;
	filter->quat.x = q.x + (mid.w * half_wx + mid.y * half_wz - mid.z * half_wy) * dt_s;
	filter->quat.y = q.y + (mid.w * half_wy - mid.x * half_wz + mid.z * half_wx) * dt_s;
	filter->quat.z = q.z + (mid.w * half_wz + mid.x * half_wy - mid.y * half_wx) * dt_s;
	normalize_quat(&filter->quat);
}
