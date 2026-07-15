#include "MPU6050.h"

#include <stddef.h>

#include "MPU6050_Reg.h"

static MPU6050_BusOps g_bus;

void MPU6050_SetBusOps(const MPU6050_BusOps *ops)
{
    if (ops == NULL)
    {
        g_bus.write_reg = NULL;
        g_bus.read_regs = NULL;
        g_bus.user_ctx = NULL;
        return;
    }
    g_bus = *ops;
}

void MPU6050_WriteReg(uint8_t reg_address, uint8_t data)
{
    if (g_bus.write_reg != NULL)
    {
        (void)g_bus.write_reg(g_bus.user_ctx, reg_address, data);
    }
}

uint8_t MPU6050_ReadReg(uint8_t reg_address)
{
    uint8_t data = 0u;

    if (g_bus.read_regs != NULL)
    {
        (void)g_bus.read_regs(g_bus.user_ctx, reg_address, &data, 1u);
    }
    return data;
}

void MPU6050_Init(void)
{
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01u);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00u);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09u);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06u);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18u);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18u);
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

int MPU6050_GetData(int16_t *acc_x,
                     int16_t *acc_y,
                     int16_t *acc_z,
                     int16_t *gyro_x,
                     int16_t *gyro_y,
                     int16_t *gyro_z)
{
    uint8_t data[14];

    if ((acc_x == NULL) || (acc_y == NULL) || (acc_z == NULL) ||
        (gyro_x == NULL) || (gyro_y == NULL) || (gyro_z == NULL) ||
        (g_bus.read_regs == NULL))
    {
        return -1;
    }
    if (g_bus.read_regs(g_bus.user_ctx,
                        MPU6050_ACCEL_XOUT_H,
                        data,
                        sizeof(data)) != 0)
    {
        return -1;
    }

    *acc_x = (int16_t)(((uint16_t)data[0] << 8u) | data[1]);
    *acc_y = (int16_t)(((uint16_t)data[2] << 8u) | data[3]);
    *acc_z = (int16_t)(((uint16_t)data[4] << 8u) | data[5]);
    *gyro_x = (int16_t)(((uint16_t)data[8] << 8u) | data[9]);
    *gyro_y = (int16_t)(((uint16_t)data[10] << 8u) | data[11]);
    *gyro_z = (int16_t)(((uint16_t)data[12] << 8u) | data[13]);
    return 0;
}
