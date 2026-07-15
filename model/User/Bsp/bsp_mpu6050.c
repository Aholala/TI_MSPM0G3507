#include "bsp_mpu6050.h"

#include "MPU6050.h"
#include "bsp_i2c.h"
#include "ti_msp_dl_config.h"

#define MPU6050_ADDRESS_7BIT 0x68u
#define MPU6050_ADDRESS_ALT_7BIT 0x69u
#define MPU6050_WHO_AM_I_VALUE 0x68u

static uint8_t g_mpu6050_address = MPU6050_ADDRESS_7BIT;

static int write_reg(void *user_ctx, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    (void)user_ctx;
    return (BspI2c_Write(MPU6050_I2C_INST,
                         g_mpu6050_address,
                         data,
                         sizeof(data)) == BSP_I2C_OK) ? 0 : -1;
}

static int read_regs(void *user_ctx,
                     uint8_t reg,
                     uint8_t *data,
                     uint16_t size)
{
    (void)user_ctx;
    return (BspI2c_WriteRead(MPU6050_I2C_INST,
                             g_mpu6050_address,
                             &reg,
                             1u,
                             data,
                             size) == BSP_I2C_OK) ? 0 : -1;
}

int BspMpu6050_Init(void)
{
    const MPU6050_BusOps ops = {
        .write_reg = write_reg,
        .read_regs = read_regs,
        .user_ctx = 0,
    };

    MPU6050_SetBusOps(&ops);
    g_mpu6050_address = MPU6050_ADDRESS_7BIT;
    MPU6050_Init();
    if (MPU6050_GetID() == MPU6050_WHO_AM_I_VALUE)
    {
        return 0;
    }

    g_mpu6050_address = MPU6050_ADDRESS_ALT_7BIT;
    MPU6050_Init();
    return (MPU6050_GetID() == MPU6050_WHO_AM_I_VALUE) ? 0 : -1;
}

uint8_t BspMpu6050_GetAddress(void)
{
    return g_mpu6050_address;
}
