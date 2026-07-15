#ifndef APP_MPU6050_H
#define APP_MPU6050_H

#include <stdint.h>

#include "lib_attitude.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_MPU6050_PERIOD_MS 10u

typedef struct
{
    imu_solution_t solution;
    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
    float raw_gyro_z_dps;
    float bias_gyro_z_dps;
    float corrected_gyro_z_dps;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float continuous_yaw_deg;
    uint32_t update_count;
    uint32_t read_error_count;
    int32_t init_status;
    uint16_t gyro_calibration_count;
    uint8_t i2c_address;
    uint8_t who_am_i;
    uint8_t initialized;
    uint8_t gyro_calibrated;
    uint8_t stationary;
} AppMpu6050_Snapshot;

extern AppMpu6050_Snapshot g_debug_mpu6050_snapshot;
extern imu_attitude_estimator_t g_debug_mpu6050_attitude;

int AppMpu6050_Init(void);
void AppMpu6050_RunOnce(void);
AppMpu6050_Snapshot AppMpu6050_GetSnapshot(void);
void AppMpu6050_ResetYaw(void);

#ifdef __cplusplus
}
#endif

#endif
