#include "app_mpu6050.h"

#include <string.h>

#include "MPU6050.h"
#include "bsp_mpu6050.h"
#include "ti_msp_dl_config.h"

#define MPU6050_ACCEL_LSB_PER_G 2048.0f
#define MPU6050_GYRO_LSB_PER_DPS 16.4f
#define STANDARD_GRAVITY_MPS2 9.80665f
#define DEG_TO_RAD 0.01745329252f

static imu_attitude_estimator_t g_attitude;
static imu_solution_t g_solution;
static AppMpu6050_Snapshot g_snapshot;
static uint16_t g_init_retry_ticks;
AppMpu6050_Snapshot g_debug_mpu6050_snapshot;
imu_attitude_estimator_t g_debug_mpu6050_attitude;

static uint32_t enter_lock(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void leave_lock(uint32_t primask)
{
    if (primask == 0u)
    {
        __enable_irq();
    }
}

static void publish_snapshot(void)
{
    uint32_t primask;

    g_snapshot.solution = g_solution;
    g_snapshot.gyro_calibrated = imu_attitude_is_gyro_calibrated(&g_attitude);
    g_snapshot.gyro_calibration_count = g_attitude.gyro_calibration_count;
    g_snapshot.bias_gyro_z_dps = g_attitude.gyro_bias_radps.z / DEG_TO_RAD;
    g_snapshot.corrected_gyro_z_dps =
        g_solution.filtered.gyro_radps.z / DEG_TO_RAD;
    g_snapshot.roll_deg = g_solution.euler.roll_deg;
    g_snapshot.pitch_deg = g_solution.euler.pitch_deg;
    g_snapshot.yaw_deg = g_solution.euler.yaw_deg;
    g_snapshot.continuous_yaw_deg =
        imu_attitude_get_continuous_yaw_deg(&g_attitude);
    g_snapshot.stationary = g_solution.compensation.stationary;
    /* The full estimator is debug-only. Copy it with interrupts enabled so
     * encoder edges cannot be lost during this relatively large structure
     * assignment. */
    g_debug_mpu6050_attitude = g_attitude;

    primask = enter_lock();
    g_debug_mpu6050_snapshot = g_snapshot;

    leave_lock(primask);
}

int AppMpu6050_Init(void)
{
    int status;

    memset(&g_snapshot, 0, sizeof(g_snapshot));
    memset(&g_solution, 0, sizeof(g_solution));
    g_init_retry_ticks = 0u;
    imu_attitude_init(&g_attitude);
    status = BspMpu6050_Init();
    g_snapshot.init_status = status;
    g_snapshot.i2c_address = BspMpu6050_GetAddress();
    g_snapshot.who_am_i = MPU6050_GetID();
    g_snapshot.initialized = (status == 0) ? 1u : 0u;
    publish_snapshot();
    return status;
}

void AppMpu6050_RunOnce(void)
{
    imu_sensor_sample_t sample;
    int16_t accel_x = 0;
    int16_t accel_y = 0;
    int16_t accel_z = 0;
    int16_t gyro_x = 0;
    int16_t gyro_y = 0;
    int16_t gyro_z = 0;

    if (g_snapshot.initialized == 0u)
    {
        /* A slowly rising sensor supply can make the first WHO_AM_I fail.
         * Retry every 500 ms without blocking the rest of the application. */
        if (++g_init_retry_ticks >= (500u / APP_MPU6050_PERIOD_MS))
        {
            int status;

            g_init_retry_ticks = 0u;
            status = BspMpu6050_Init();
            g_snapshot.init_status = status;
            g_snapshot.i2c_address = BspMpu6050_GetAddress();
            g_snapshot.who_am_i = MPU6050_GetID();
            g_snapshot.initialized = (status == 0) ? 1u : 0u;
        }
        publish_snapshot();
        return;
    }

    if (MPU6050_GetData(&accel_x, &accel_y, &accel_z,
                        &gyro_x, &gyro_y, &gyro_z) != 0)
    {
        ++g_snapshot.read_error_count;
        publish_snapshot();
        return;
    }
    g_snapshot.raw_accel_x = accel_x;
    g_snapshot.raw_accel_y = accel_y;
    g_snapshot.raw_accel_z = accel_z;
    g_snapshot.raw_gyro_x = gyro_x;
    g_snapshot.raw_gyro_y = gyro_y;
    g_snapshot.raw_gyro_z = gyro_z;
    g_snapshot.raw_gyro_z_dps = (float)gyro_z / MPU6050_GYRO_LSB_PER_DPS;

    sample.accel_mps2.x = ((float)accel_x / MPU6050_ACCEL_LSB_PER_G) *
                          STANDARD_GRAVITY_MPS2;
    sample.accel_mps2.y = ((float)accel_y / MPU6050_ACCEL_LSB_PER_G) *
                          STANDARD_GRAVITY_MPS2;
    sample.accel_mps2.z = ((float)accel_z / MPU6050_ACCEL_LSB_PER_G) *
                          STANDARD_GRAVITY_MPS2;
    sample.gyro_radps.x = ((float)gyro_x / MPU6050_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    sample.gyro_radps.y = ((float)gyro_y / MPU6050_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    sample.gyro_radps.z = ((float)gyro_z / MPU6050_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    sample.temp_degc = 0.0f;
    sample.timestamp_us = g_snapshot.update_count * APP_MPU6050_PERIOD_MS * 1000u;

    imu_attitude_update(&g_attitude, &sample,
                        (float)APP_MPU6050_PERIOD_MS / 1000.0f,
                        &g_solution);
    ++g_snapshot.update_count;
    publish_snapshot();
}

AppMpu6050_Snapshot AppMpu6050_GetSnapshot(void)
{
    AppMpu6050_Snapshot snapshot;
    uint32_t primask = enter_lock();

    snapshot = g_snapshot;
    leave_lock(primask);
    return snapshot;
}

void AppMpu6050_ResetYaw(void)
{
    imu_attitude_reset_yaw(&g_attitude);
    publish_snapshot();
}
