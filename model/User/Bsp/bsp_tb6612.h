#ifndef BSP_TB6612_H
#define BSP_TB6612_H

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

/* Initialize both PWM channels stopped and both bridges in coast mode. */
void BspTb6612_Init(void);
void BspTb6612_BindMotor(BspTb6612_MotorId motor_id, DcMotor_Ops *ops);

#ifdef __cplusplus
}
#endif

#endif
