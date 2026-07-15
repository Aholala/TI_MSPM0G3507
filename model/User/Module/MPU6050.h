#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

typedef struct
{
	int (*write_reg)(void *user_ctx, uint8_t reg, uint8_t value);
	int (*read_regs)(void *user_ctx, uint8_t reg, uint8_t *data, uint16_t size);
	void *user_ctx;
} MPU6050_BusOps;

void MPU6050_SetBusOps(const MPU6050_BusOps *ops);

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
int MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif
