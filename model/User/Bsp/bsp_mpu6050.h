#ifndef BSP_MPU6050_H
#define BSP_MPU6050_H

#include <stdint.h>

/* Attach MPU6050 to I2C1 and configure its registers. Returns 0 on success. */
int BspMpu6050_Init(void);
uint8_t BspMpu6050_GetAddress(void);

#endif
