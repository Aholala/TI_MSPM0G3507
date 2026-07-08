/**
 * @file bsp_tb6612.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef BSP_TB6612_H
#define BSP_TB6612_H

#include "board_config.h"
#include "module_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BSP_TB6612_MOTOR_LEFT = 0,
    BSP_TB6612_MOTOR_RIGHT,
    BSP_TB6612_MOTOR_COUNT
} BspTb6612_MotorId;

void BspTb6612_Init(void);
void BspTb6612_BindMotor(BspTb6612_MotorId motor_id, DcMotor_Ops *ops);

#ifdef __cplusplus
}
#endif

#endif
