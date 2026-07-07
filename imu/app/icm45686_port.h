#ifndef __ICM45686_PORT_H
#define __ICM45686_PORT_H

/*
 * Compatibility API.
 * New code should include icm45686_app.h, icm45686_module.h, or icm45686_bsp.h.
 */

int setup_imu(int use_ln, int accel_en, int gyro_en);
int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc);

#endif
