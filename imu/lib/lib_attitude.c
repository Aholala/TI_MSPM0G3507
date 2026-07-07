#include "lib_attitude.h"

#include <math.h>
#include <string.h>

#ifndef IMU_PI
#define IMU_PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG (180.0f / IMU_PI)

static float clampf_local(float value, float min_value, float max_value)
{
	if (value < min_value)
		return min_value;
	if (value > max_value)
		return max_value;
	return value;
}

static float wrap_pi(float angle)
{
	while (angle > IMU_PI)
		angle -= 2.0f * IMU_PI;
	while (angle < -IMU_PI)
		angle += 2.0f * IMU_PI;
	return angle;
}

static void normalize_quat(imu_quat_t *q)
{
	float norm;

	if (!q)
		return;

	norm = sqrtf((q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z));
	if (norm <= 0.0f) {
		q->w = 1.0f;
		q->x = 0.0f;
		q->y = 0.0f;
		q->z = 0.0f;
		return;
	}

	q->w /= norm;
	q->x /= norm;
	q->y /= norm;
	q->z /= norm;
}

static float vec3_norm(imu_vec3_t v)
{
	return sqrtf((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
}

static imu_vec3_t vec3_sub(imu_vec3_t a, imu_vec3_t b)
{
	imu_vec3_t out;

	out.x = a.x - b.x;
	out.y = a.y - b.y;
	out.z = a.z - b.z;

	return out;
}

static void update_gyro_bias(imu_attitude_estimator_t *estimator, const imu_sensor_sample_t *sample,
                             float accel_norm, imu_solution_t *solution)
{
	float accel_error;
	float gyro_norm;
	float alpha;
	uint8_t stationary;

	accel_error = fabsf(accel_norm - estimator->config.gravity_mps2);
	gyro_norm = vec3_norm(sample->gyro_radps);
	stationary = (accel_error < estimator->config.stationary_accel_tolerance_mps2) &&
	             (gyro_norm < estimator->config.stationary_gyro_threshold_radps);

	if (stationary) {
		alpha = estimator->config.gyro_bias_alpha;
		estimator->gyro_bias_radps.x =
		    ((1.0f - alpha) * estimator->gyro_bias_radps.x) + (alpha * sample->gyro_radps.x);
		estimator->gyro_bias_radps.y =
		    ((1.0f - alpha) * estimator->gyro_bias_radps.y) + (alpha * sample->gyro_radps.y);
		estimator->gyro_bias_radps.z =
		    ((1.0f - alpha) * estimator->gyro_bias_radps.z) + (alpha * sample->gyro_radps.z);
	}

	solution->compensation.gyro_bias_radps = estimator->gyro_bias_radps;
	solution->compensation.stationary = stationary;
}

imu_quat_t imu_quat_from_euler_rad(float roll_rad, float pitch_rad, float yaw_rad)
{
	float cr = cosf(roll_rad * 0.5f);
	float sr = sinf(roll_rad * 0.5f);
	float cp = cosf(pitch_rad * 0.5f);
	float sp = sinf(pitch_rad * 0.5f);
	float cy = cosf(yaw_rad * 0.5f);
	float sy = sinf(yaw_rad * 0.5f);
	imu_quat_t q;

	q.w = (cr * cp * cy) + (sr * sp * sy);
	q.x = (sr * cp * cy) - (cr * sp * sy);
	q.y = (cr * sp * cy) + (sr * cp * sy);
	q.z = (cr * cp * sy) - (sr * sp * cy);
	normalize_quat(&q);

	return q;
}

imu_euler_t imu_euler_from_quat(const imu_quat_t *quat)
{
	float sinr_cosp;
	float cosr_cosp;
	float sinp;
	float siny_cosp;
	float cosy_cosp;
	imu_euler_t euler = {0};

	if (!quat)
		return euler;

	sinr_cosp = 2.0f * ((quat->w * quat->x) + (quat->y * quat->z));
	cosr_cosp = 1.0f - (2.0f * ((quat->x * quat->x) + (quat->y * quat->y)));
	euler.roll_deg = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

	sinp = 2.0f * ((quat->w * quat->y) - (quat->z * quat->x));
	if (sinp >= 1.0f)
		euler.pitch_deg = 90.0f;
	else if (sinp <= -1.0f)
		euler.pitch_deg = -90.0f;
	else
		euler.pitch_deg = asinf(sinp) * RAD_TO_DEG;

	siny_cosp = 2.0f * ((quat->w * quat->z) + (quat->x * quat->y));
	cosy_cosp = 1.0f - (2.0f * ((quat->y * quat->y) + (quat->z * quat->z)));
	euler.yaw_deg = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;

	return euler;
}

void imu_attitude_init(imu_attitude_estimator_t *estimator)
{
	imu_attitude_config_t config;

	imu_attitude_get_default_config(&config);
	imu_attitude_init_with_config(estimator, &config);
}

void imu_attitude_get_default_config(imu_attitude_config_t *config)
{
	if (!config)
		return;

	config->gravity_mps2 = 9.80665f;
	config->accel_trust_min = 0.02f;
	config->accel_trust_max = 1.0f;
	config->accel_norm_tolerance_mps2 = 3.0f;
	config->stationary_gyro_threshold_radps = 0.035f;
	config->stationary_accel_tolerance_mps2 = 0.35f;
	config->gyro_bias_alpha = 0.002f;
	config->yaw_gyro_deadband_radps = 0.003f;
	config->angle_r_min = 0.02f;
	config->angle_r_max = 2.0f;
}

void imu_attitude_init_with_config(imu_attitude_estimator_t *estimator,
                                   const imu_attitude_config_t *config)
{
	if (!estimator)
		return;

	memset(estimator, 0, sizeof(*estimator));
	if (config)
		estimator->config = *config;
	else
		imu_attitude_get_default_config(&estimator->config);

	imu_sensor_kalman_init(&estimator->sensor_filter);
	kalman_angle_init(&estimator->roll_filter, 0.001f, 0.003f, estimator->config.angle_r_min,
	                  0.0f);
	kalman_angle_init(&estimator->pitch_filter, 0.001f, 0.003f, estimator->config.angle_r_min,
	                  0.0f);
	estimator->initialized = 1;
}

void imu_attitude_update(imu_attitude_estimator_t *estimator, const imu_sensor_sample_t *raw,
                         float dt_s, imu_solution_t *solution)
{
	float roll_acc;
	float pitch_acc;
	float roll_rad;
	float pitch_rad;
	float accel_norm;
	float accel_error;
	float accel_trust;
	float corrected_yaw_rate;

	if (!estimator || !raw || !solution)
		return;

	if (!estimator->initialized)
		imu_attitude_init(estimator);

	if (dt_s <= 0.0f)
		dt_s = 0.005f;

	imu_sensor_kalman_update(&estimator->sensor_filter, raw, &solution->filtered);
	solution->raw = *raw;

	accel_norm = vec3_norm(solution->filtered.accel_mps2);
	accel_error = fabsf(accel_norm - estimator->config.gravity_mps2);
	accel_trust = 1.0f - (accel_error / estimator->config.accel_norm_tolerance_mps2);
	accel_trust = clampf_local(accel_trust, estimator->config.accel_trust_min,
	                           estimator->config.accel_trust_max);
	solution->compensation.accel_trust = accel_trust;

	update_gyro_bias(estimator, &solution->filtered, accel_norm, solution);
	solution->filtered.gyro_radps = vec3_sub(solution->filtered.gyro_radps,
	                                         estimator->gyro_bias_radps);

	estimator->roll_filter.r_measure =
	    estimator->config.angle_r_min +
	    ((estimator->config.angle_r_max - estimator->config.angle_r_min) * (1.0f - accel_trust));
	estimator->pitch_filter.r_measure = estimator->roll_filter.r_measure;

	roll_acc = atan2f(solution->filtered.accel_mps2.y, solution->filtered.accel_mps2.z);
	pitch_acc = atan2f(-solution->filtered.accel_mps2.x,
	                   sqrtf((solution->filtered.accel_mps2.y * solution->filtered.accel_mps2.y) +
	                         (solution->filtered.accel_mps2.z * solution->filtered.accel_mps2.z)));

	roll_rad = kalman_angle_update(&estimator->roll_filter, roll_acc,
	                               solution->filtered.gyro_radps.x, dt_s);
	pitch_rad = kalman_angle_update(&estimator->pitch_filter, pitch_acc,
	                                solution->filtered.gyro_radps.y, dt_s);
	corrected_yaw_rate = solution->filtered.gyro_radps.z;
	if (fabsf(corrected_yaw_rate) < estimator->config.yaw_gyro_deadband_radps)
		corrected_yaw_rate = 0.0f;
	estimator->yaw_rad = wrap_pi(estimator->yaw_rad + (corrected_yaw_rate * dt_s));

	solution->quat = imu_quat_from_euler_rad(roll_rad, pitch_rad, estimator->yaw_rad);
	solution->euler = imu_euler_from_quat(&solution->quat);
}
