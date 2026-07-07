/**
 * @file icm45686_port.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __ICM45686_PORT_H
#define __ICM45686_PORT_H

/*
 * Compatibility API.
 * New code should include app_icm45686.h, module_icm45686.h, or bsp_icm45686.h.
 */

int setup_imu(int use_ln, int accel_en, int gyro_en);
int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc);

#endif
