#include "icm45686_port.h"

#include "icm45686_app.h"

int setup_imu(int use_ln, int accel_en, int gyro_en)
{
	icm45686_config_t config;

	icm45686_module_get_default_config(&config);
	config.use_ln   = use_ln;
	config.accel_en = accel_en;
	config.gyro_en  = gyro_en;

	return icm45686_app_init(&config);
}

int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc)
{
	int rc;
	icm45686_data_t data;

	if (!accel_mg || !gyro_dps || !temp_degc)
		return INV_IMU_ERROR_BAD_ARG;

	rc = icm45686_app_read_data_if_ready(&data);
	if (rc)
		return rc;

	accel_mg[0] = data.accel_mg[0];
	accel_mg[1] = data.accel_mg[1];
	accel_mg[2] = data.accel_mg[2];
	gyro_dps[0] = data.gyro_dps[0];
	gyro_dps[1] = data.gyro_dps[1];
	gyro_dps[2] = data.gyro_dps[2];
	*temp_degc  = data.temp_degc;

	return 0;
}
