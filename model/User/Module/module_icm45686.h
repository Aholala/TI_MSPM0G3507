/**
 * @file module_icm45686.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __ICM45686_MODULE_H
#define __ICM45686_MODULE_H

#include <stdint.h>

#include "../bsp/bsp_icm45686.h"
#include "inv_imu_driver.h"

typedef struct {
	float accel_mg[3];
	float gyro_dps[3];
	float temp_degc;
} icm45686_data_t;

typedef struct {
	int use_ln;
	int accel_en;
	int gyro_en;
	accel_config0_accel_ui_fs_sel_t accel_fsr;
	gyro_config0_gyro_ui_fs_sel_t gyro_fsr;
	accel_config0_accel_odr_t accel_odr;
	gyro_config0_gyro_odr_t gyro_odr;
	ipreg_sys2_reg_131_accel_ui_lpfbw_t accel_bw;
	ipreg_sys1_reg_172_gyro_ui_lpfbw_sel_t gyro_bw;
} icm45686_config_t;

typedef struct {
	inv_imu_device_t inv;
	const icm45686_bsp_t *bsp;
	icm45686_config_t config;
	volatile uint8_t data_ready;
	uint8_t initialized;
} icm45686_module_t;

void icm45686_module_get_default_config(icm45686_config_t *config);
int icm45686_module_init(icm45686_module_t *module, const icm45686_bsp_t *bsp,
                         const icm45686_config_t *config);
void icm45686_module_int1_irq_handler(icm45686_module_t *module);
uint8_t icm45686_module_is_data_ready(const icm45686_module_t *module);
int icm45686_module_read_data(icm45686_module_t *module, icm45686_data_t *data);
int icm45686_module_read_data_if_ready(icm45686_module_t *module, icm45686_data_t *data);
inv_imu_device_t *icm45686_module_get_inv_device(icm45686_module_t *module);

#endif
