#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stdint.h>

#include "ti_msp_dl_config.h"

typedef enum
{
    BSP_I2C_OK = 0,
    BSP_I2C_ERROR,
    BSP_I2C_TIMEOUT,
    BSP_I2C_INVALID_ARG
} BspI2c_Status;

/* Blocking primitives intended for device initialization and short transfers. */
BspI2c_Status BspI2c_Write(I2C_Regs *i2c,
                           uint8_t address_7bit,
                           const uint8_t *data,
                           uint16_t size);
BspI2c_Status BspI2c_WriteRead(I2C_Regs *i2c,
                               uint8_t address_7bit,
                               const uint8_t *write_data,
                               uint16_t write_size,
                               uint8_t *read_data,
                               uint16_t read_size);

#endif
