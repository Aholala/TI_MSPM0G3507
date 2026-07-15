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

float imu_angle_wrap_deg(float angle)
{
	while (angle > 180.0f)
		angle -= 360.0f;
	while (angle < -180.0f)
		angle += 360.0f;
	return angle;
}

static float angle_lpf_deg(float previous, float input, float alpha)
{
	return imu_angle_wrap_deg(previous + alpha * imu_angle_wrap_deg(input - previous));
}

float imu_angle_error_deg(float target_deg, float current_deg)
{
	return imu_angle_wrap_deg(target_deg - current_deg);
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
	config->yaw_gyro_scale = 1.0f;
	config->angle_r_min = 0.02f;
	config->angle_r_max = 2.0f;
	config->euler_lpf_time_constant_s = 0.03f;
	config->gyro_calibration_samples = 400U;
}

void imu_attitude_init_with_config(imu_attitude_estimator_t *estimator,
                                   const imu_attitude_config_t *config)
{
	adaptive_mahony_config_t fusion_config;

	if (!estimator)
		return;

	memset(estimator, 0, sizeof(*estimator));
	if (config)
		estimator->config = *config;
	else
		imu_attitude_get_default_config(&estimator->config);

	imu_sensor_kalman_init(&estimator->sensor_filter);
	adaptive_mahony_get_default_config(&fusion_config);
	adaptive_mahony_init(&estimator->fusion, &fusion_config);
	kalman_angle_init(&estimator->roll_filter, 0.001f, 0.003f, estimator->config.angle_r_min,
	                  0.0f);
	kalman_angle_init(&estimator->pitch_filter, 0.001f, 0.003f, estimator->config.angle_r_min,
	                  0.0f);
	/* Let the first accelerometer attitude seed both angle filters. */
	estimator->roll_filter.initialized = 0U;
	estimator->pitch_filter.initialized = 0U;
	estimator->gyro_calibrated = (estimator->config.gyro_calibration_samples == 0U);
	estimator->initialized = 1;
}

void imu_attitude_reset_yaw(imu_attitude_estimator_t *estimator)
{
	if (!estimator)
		return;

	estimator->yaw_rad = 0.0f;
	estimator->continuous_yaw_rad = 0.0f;
	estimator->previous_yaw_rate_radps = 0.0f;
	estimator->previous_wrapped_yaw_deg = 0.0f;
	estimator->yaw_unwrap_initialized = 1U;
	estimator->filtered_euler.yaw_deg = 0.0f;
	adaptive_mahony_reset_heading(&estimator->fusion);
}

uint8_t imu_attitude_is_gyro_calibrated(const imu_attitude_estimator_t *estimator)
{
	return estimator ? estimator->gyro_calibrated : 0U;
}

float imu_attitude_get_continuous_yaw_deg(const imu_attitude_estimator_t *estimator)
{
	return estimator ? (estimator->continuous_yaw_rad * RAD_TO_DEG) : 0.0f;
}

void imu_attitude_update(imu_attitude_estimator_t *estimator, const imu_sensor_sample_t *raw,
                         float dt_s, imu_solution_t *solution)
{
	float accel_norm;
	float accel_error;
	float accel_trust;
	float corrected_yaw_rate;
	float euler_lpf_alpha;
	imu_euler_t fused_euler;
	uint8_t calibration_stationary;

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

	/* A gyro-only yaw has no absolute reference. Estimate its zero-rate output
	 * while the car is stationary and do not integrate yaw until the estimate
	 * is valid. Any movement restarts the consecutive-sample calibration. */
	if (!estimator->gyro_calibrated) {
		calibration_stationary =
		    (accel_error < estimator->config.stationary_accel_tolerance_mps2) &&
		    (vec3_norm(raw->gyro_radps) < estimator->config.stationary_gyro_threshold_radps);

		if (calibration_stationary) {
			estimator->gyro_calibration_sum.x += raw->gyro_radps.x;
			estimator->gyro_calibration_sum.y += raw->gyro_radps.y;
			estimator->gyro_calibration_sum.z += raw->gyro_radps.z;
			estimator->gyro_calibration_sum_sq.x += raw->gyro_radps.x * raw->gyro_radps.x;
			estimator->gyro_calibration_sum_sq.y += raw->gyro_radps.y * raw->gyro_radps.y;
			estimator->gyro_calibration_sum_sq.z += raw->gyro_radps.z * raw->gyro_radps.z;
			++estimator->gyro_calibration_count;
		} else {
			memset(&estimator->gyro_calibration_sum, 0,
			       sizeof(estimator->gyro_calibration_sum));
			memset(&estimator->gyro_calibration_sum_sq, 0,
			       sizeof(estimator->gyro_calibration_sum_sq));
			estimator->gyro_calibration_count = 0U;
		}

		if (estimator->gyro_calibration_count >= estimator->config.gyro_calibration_samples) {
			float sample_count = (float)estimator->gyro_calibration_count;

			estimator->gyro_bias_radps.x = estimator->gyro_calibration_sum.x / sample_count;
			estimator->gyro_bias_radps.y = estimator->gyro_calibration_sum.y / sample_count;
			estimator->gyro_bias_radps.z = estimator->gyro_calibration_sum.z / sample_count;
			/* Use the measured stationary variance as each adaptive Kalman
			 * channel's R instead of carrying an MPU6050-specific constant. */
			estimator->fusion.gyro_filter[0].r = fmaxf(
			    estimator->gyro_calibration_sum_sq.x / sample_count -
			        estimator->gyro_bias_radps.x * estimator->gyro_bias_radps.x,
			    1.0e-9f);
			estimator->fusion.gyro_filter[1].r = fmaxf(
			    estimator->gyro_calibration_sum_sq.y / sample_count -
			        estimator->gyro_bias_radps.y * estimator->gyro_bias_radps.y,
			    1.0e-9f);
			estimator->fusion.gyro_filter[2].r = fmaxf(
			    estimator->gyro_calibration_sum_sq.z / sample_count -
			        estimator->gyro_bias_radps.z * estimator->gyro_bias_radps.z,
			    1.0e-9f);
			for (uint8_t axis = 0U; axis < 3U; ++axis) {
				estimator->fusion.gyro_filter[axis].initialized = 0U;
				estimator->fusion.gyro_filter[axis].index = 0U;
				estimator->fusion.gyro_filter[axis].full = 0U;
			}
			estimator->gyro_calibrated = 1U;
			imu_attitude_reset_yaw(estimator);
		}
	}

	if (estimator->gyro_calibrated)
		update_gyro_bias(estimator, raw, accel_norm, solution);
	else {
		solution->compensation.gyro_bias_radps = estimator->gyro_bias_radps;
		solution->compensation.stationary = calibration_stationary;
	}
	/* The adaptive Mahony core owns gyro filtering. Keep the existing scalar
	 * Kalman stage only for acceleration, then feed bias-corrected raw rates to
	 * the innovation-adaptive gyro filters to avoid double filtering and lag. */
	solution->filtered.gyro_radps = vec3_sub(raw->gyro_radps, estimator->gyro_bias_radps);
	solution->filtered.gyro_radps.z *= estimator->config.yaw_gyro_scale;
	corrected_yaw_rate = solution->filtered.gyro_radps.z;
	if (fabsf(corrected_yaw_rate) < estimator->config.yaw_gyro_deadband_radps)
		corrected_yaw_rate = 0.0f;
	/* Roll and pitch must remain responsive while the startup bias estimate is
	 * being collected.  Only yaw has no accelerometer reference, so freeze its
	 * integration until the stationary Z-axis bias is valid. */
	if (!estimator->gyro_calibrated)
		corrected_yaw_rate = 0.0f;
	solution->filtered.gyro_radps.z = corrected_yaw_rate;

	adaptive_mahony_update(&estimator->fusion, solution->filtered.accel_mps2,
	                       solution->filtered.gyro_radps, accel_trust,
	                       solution->compensation.stationary, dt_s);
	solution->filtered.gyro_radps = estimator->fusion.filtered_gyro_radps;
	corrected_yaw_rate = solution->filtered.gyro_radps.z;

	estimator->previous_yaw_rate_radps = corrected_yaw_rate;

	solution->quat = estimator->fusion.quat;
	fused_euler = imu_euler_from_quat(&solution->quat);
	if (!estimator->euler_lpf_initialized ||
	    estimator->config.euler_lpf_time_constant_s <= 0.0f) {
		estimator->filtered_euler = fused_euler;
		estimator->euler_lpf_initialized = 1U;
	} else {
		euler_lpf_alpha = dt_s / (estimator->config.euler_lpf_time_constant_s + dt_s);
		euler_lpf_alpha = clampf_local(euler_lpf_alpha, 0.0f, 1.0f);
		estimator->filtered_euler.roll_deg =
		    angle_lpf_deg(estimator->filtered_euler.roll_deg, fused_euler.roll_deg,
		                  euler_lpf_alpha);
		estimator->filtered_euler.pitch_deg =
		    angle_lpf_deg(estimator->filtered_euler.pitch_deg, fused_euler.pitch_deg,
		                  euler_lpf_alpha);
		estimator->filtered_euler.yaw_deg =
		    angle_lpf_deg(estimator->filtered_euler.yaw_deg, fused_euler.yaw_deg,
		                  euler_lpf_alpha);
	}
	solution->euler = estimator->filtered_euler;
	/* Unwrap the filtered quaternion yaw. Crossing +180/-180 now contributes
	 * only the real small angular step, never an artificial 360-degree jump. */
	if (!estimator->yaw_unwrap_initialized) {
		estimator->previous_wrapped_yaw_deg = solution->euler.yaw_deg;
		estimator->yaw_unwrap_initialized = 1U;
	} else if (estimator->gyro_calibrated) {
		float yaw_step_deg = imu_angle_wrap_deg(
		    solution->euler.yaw_deg - estimator->previous_wrapped_yaw_deg);
		estimator->continuous_yaw_rad += yaw_step_deg / RAD_TO_DEG;
		estimator->previous_wrapped_yaw_deg = solution->euler.yaw_deg;
	} else {
		estimator->continuous_yaw_rad = 0.0f;
		estimator->previous_wrapped_yaw_deg = solution->euler.yaw_deg;
	}
	estimator->yaw_rad = solution->euler.yaw_deg / RAD_TO_DEG;
}
