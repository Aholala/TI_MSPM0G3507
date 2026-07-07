#include "icm45686_module.h"

#include <stddef.h>
#include <string.h>

static icm45686_module_t *s_active_module;

static int module_check_rc(int rc)
{
	return rc;
}

static void module_sleep_us(uint32_t us)
{
	if (s_active_module && s_active_module->bsp && s_active_module->bsp->sleep_us)
		s_active_module->bsp->sleep_us(s_active_module->bsp->ctx, us);
}

static int module_spi_read_reg(uint8_t reg, uint8_t *buf, uint32_t len)
{
	uint8_t addr = reg | 0x80;
	const icm45686_bsp_t *bsp;
	int rc;

	if (!s_active_module || !s_active_module->bsp || !buf)
		return INV_IMU_ERROR_BAD_ARG;

	bsp = s_active_module->bsp;
	if (!bsp->cs_low || !bsp->cs_high || !bsp->spi_transfer)
		return INV_IMU_ERROR_BAD_ARG;

	bsp->cs_low(bsp->ctx);
	rc = bsp->spi_transfer(bsp->ctx, &addr, 0, 1);
	if (rc == 0)
		rc = bsp->spi_transfer(bsp->ctx, 0, buf, len);
	bsp->cs_high(bsp->ctx);

	return rc;
}

static int module_spi_write_reg(uint8_t reg, const uint8_t *buf, uint32_t len)
{
	const icm45686_bsp_t *bsp;
	int rc;

	if (!s_active_module || !s_active_module->bsp || !buf)
		return INV_IMU_ERROR_BAD_ARG;

	bsp = s_active_module->bsp;
	if (!bsp->cs_low || !bsp->cs_high || !bsp->spi_transfer)
		return INV_IMU_ERROR_BAD_ARG;

	bsp->cs_low(bsp->ctx);
	rc = bsp->spi_transfer(bsp->ctx, &reg, 0, 1);
	if (rc == 0)
		rc = bsp->spi_transfer(bsp->ctx, buf, 0, len);
	bsp->cs_high(bsp->ctx);

	return rc;
}

static void module_bsp_int1_callback(void *arg)
{
	icm45686_module_int1_irq_handler((icm45686_module_t *)arg);
}

static float accel_fsr_g(accel_config0_accel_ui_fs_sel_t fsr)
{
	switch (fsr) {
	case ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G:
		return 2.0f;
	case ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G:
		return 4.0f;
	case ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G:
		return 8.0f;
	case ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G:
		return 16.0f;
#if INV_IMU_HIGH_FSR_SUPPORTED
	case ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G:
		return 32.0f;
#endif
	default:
		return 4.0f;
	}
}

static float gyro_fsr_dps(gyro_config0_gyro_ui_fs_sel_t fsr)
{
	switch (fsr) {
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_15_625_DPS:
		return 15.625f;
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_31_25_DPS:
		return 31.25f;
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_62_5_DPS:
		return 62.5f;
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_125_DPS:
		return 125.0f;
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_250_DPS:
		return 250.0f;
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS:
		return 500.0f;
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS:
		return 1000.0f;
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS:
		return 2000.0f;
#if INV_IMU_HIGH_FSR_SUPPORTED
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS:
		return 4000.0f;
#endif
	default:
		return 1000.0f;
	}
}

void icm45686_module_get_default_config(icm45686_config_t *config)
{
	if (!config)
		return;

	config->use_ln    = 1;
	config->accel_en  = 1;
	config->gyro_en   = 1;
	config->accel_fsr = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G;
	config->gyro_fsr  = GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS;
	config->accel_odr = ACCEL_CONFIG0_ACCEL_ODR_200_HZ;
	config->gyro_odr  = GYRO_CONFIG0_GYRO_ODR_200_HZ;
	config->accel_bw  = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4;
	config->gyro_bw   = IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4;
}

int icm45686_module_init(icm45686_module_t *module, const icm45686_bsp_t *bsp,
                         const icm45686_config_t *config)
{
	int rc = INV_IMU_OK;
	uint8_t whoami = 0;
	inv_imu_int_pin_config_t int_pin_config;
	inv_imu_int_state_t int_config;
	drive_config0_t drive_config0;

	if (!module || !bsp)
		return INV_IMU_ERROR_BAD_ARG;

	memset(module, 0, sizeof(*module));
	module->bsp = bsp;
	if (config)
		module->config = *config;
	else
		icm45686_module_get_default_config(&module->config);

	s_active_module = module;

	if (bsp->init) {
		rc = bsp->init(bsp->ctx);
		if (module_check_rc(rc))
			return rc;
	}

	if (bsp->int1_init) {
		rc = bsp->int1_init(bsp->ctx, module_bsp_int1_callback, module);
		if (module_check_rc(rc))
			return rc;
	}

	module->inv.transport.read_reg   = module_spi_read_reg;
	module->inv.transport.write_reg  = module_spi_write_reg;
	module->inv.transport.sleep_us   = module_sleep_us;
	module->inv.transport.serif_type = UI_SPI4;

	module_sleep_us(3000);

	drive_config0.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS;
	rc |= inv_imu_write_reg(&module->inv, DRIVE_CONFIG0, 1, (uint8_t *)&drive_config0);
	if (module_check_rc(rc))
		return rc;
	module_sleep_us(2);

	rc |= inv_imu_get_who_am_i(&module->inv, &whoami);
	if (module_check_rc(rc))
		return rc;
	if (whoami != INV_IMU_WHOAMI)
		return INV_IMU_ERROR;

	rc |= inv_imu_soft_reset(&module->inv);
	if (module_check_rc(rc))
		return rc;

	int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
	int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
	int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
	rc |= inv_imu_set_pin_config_int(&module->inv, INV_IMU_INT1, &int_pin_config);
	if (module_check_rc(rc))
		return rc;

	memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
	int_config.INV_UI_DRDY = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(&module->inv, INV_IMU_INT1, &int_config);
	if (module_check_rc(rc))
		return rc;

	rc |= inv_imu_set_accel_fsr(&module->inv, module->config.accel_fsr);
	rc |= inv_imu_set_gyro_fsr(&module->inv, module->config.gyro_fsr);
	if (module_check_rc(rc))
		return rc;

	rc |= inv_imu_set_accel_frequency(&module->inv, module->config.accel_odr);
	rc |= inv_imu_set_gyro_frequency(&module->inv, module->config.gyro_odr);
	if (module_check_rc(rc))
		return rc;

	rc |= inv_imu_set_accel_ln_bw(&module->inv, module->config.accel_bw);
	rc |= inv_imu_set_gyro_ln_bw(&module->inv, module->config.gyro_bw);
	if (module_check_rc(rc))
		return rc;

	rc |= inv_imu_select_accel_lp_clk(&module->inv, SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC);
	if (module_check_rc(rc))
		return rc;

	if (module->config.use_ln) {
		if (module->config.accel_en)
			rc |= inv_imu_set_accel_mode(&module->inv, PWR_MGMT0_ACCEL_MODE_LN);
		if (module->config.gyro_en)
			rc |= inv_imu_set_gyro_mode(&module->inv, PWR_MGMT0_GYRO_MODE_LN);
	} else {
		if (module->config.accel_en)
			rc |= inv_imu_set_accel_mode(&module->inv, PWR_MGMT0_ACCEL_MODE_LP);
		if (module->config.gyro_en)
			rc |= inv_imu_set_gyro_mode(&module->inv, PWR_MGMT0_GYRO_MODE_LP);
	}
	if (module_check_rc(rc))
		return rc;

	module->initialized = 1;
	module->data_ready = 0;

	return INV_IMU_OK;
}

void icm45686_module_int1_irq_handler(icm45686_module_t *module)
{
	if (module)
		module->data_ready = 1;
}

uint8_t icm45686_module_is_data_ready(const icm45686_module_t *module)
{
	return module ? module->data_ready : 0;
}

int icm45686_module_read_data(icm45686_module_t *module, icm45686_data_t *data)
{
	int rc;
	float acc_fsr;
	float gyr_fsr;
	inv_imu_sensor_data_t raw;

	if (!module || !data || !module->initialized)
		return INV_IMU_ERROR_BAD_ARG;

	s_active_module = module;
	rc = inv_imu_get_register_data(&module->inv, &raw);
	if (module_check_rc(rc))
		return rc;

	acc_fsr = accel_fsr_g(module->config.accel_fsr);
	gyr_fsr = gyro_fsr_dps(module->config.gyro_fsr);

	data->accel_mg[0] = ((float)raw.accel_data[0] * acc_fsr * 1000.0f) / 32768.0f;
	data->accel_mg[1] = ((float)raw.accel_data[1] * acc_fsr * 1000.0f) / 32768.0f;
	data->accel_mg[2] = ((float)raw.accel_data[2] * acc_fsr * 1000.0f) / 32768.0f;
	data->gyro_dps[0] = ((float)raw.gyro_data[0] * gyr_fsr) / 32768.0f;
	data->gyro_dps[1] = ((float)raw.gyro_data[1] * gyr_fsr) / 32768.0f;
	data->gyro_dps[2] = ((float)raw.gyro_data[2] * gyr_fsr) / 32768.0f;
	data->temp_degc   = 25.0f + ((float)raw.temp_data / 128.0f);

	module->data_ready = 0;

	return INV_IMU_OK;
}

int icm45686_module_read_data_if_ready(icm45686_module_t *module, icm45686_data_t *data)
{
	if (!icm45686_module_is_data_ready(module))
		return INV_IMU_ERROR_EDMP_BUF_EMPTY;

	return icm45686_module_read_data(module, data);
}

inv_imu_device_t *icm45686_module_get_inv_device(icm45686_module_t *module)
{
	return module ? &module->inv : 0;
}
