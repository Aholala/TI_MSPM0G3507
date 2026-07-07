/**
 * @file module_motor.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "module_motor.h"

#include <stddef.h>

static int16_t clamp_speed(int16_t speed)
{
    if (speed > DC_MOTOR_MAX_SPEED)
    {
        return DC_MOTOR_MAX_SPEED;
    }

    if (speed < -DC_MOTOR_MAX_SPEED)
    {
        return -DC_MOTOR_MAX_SPEED;
    }

    return speed;
}

static uint16_t abs_speed(int16_t speed)
{
    return (uint16_t)((speed < 0) ? -speed : speed);
}

void DcMotor_Init(DcMotor *motor, const DcMotor_Ops *ops, const DcMotor_Config *cfg)
{
    if (motor == NULL)
    {
        return;
    }

    motor->ops.set_mode = NULL;
    motor->ops.set_duty = NULL;
    motor->ops.user_ctx = NULL;
    if (ops != NULL)
    {
        motor->ops = *ops;
    }

    motor->cfg.inverted = 0u;
    if (cfg != NULL)
    {
        motor->cfg = *cfg;
    }

    motor->speed = 0;
    DcMotor_Coast(motor);
}

void DcMotor_SetSpeed(DcMotor *motor, int16_t speed)
{
    DcMotor_Mode mode;

    if (motor == NULL)
    {
        return;
    }

    speed = clamp_speed(speed);
    motor->speed = speed;

    if (motor->ops.set_duty == NULL || motor->ops.set_mode == NULL)
    {
        return;
    }

    if (speed == 0)
    {
        motor->ops.set_duty(motor->ops.user_ctx, 0u);
        motor->ops.set_mode(motor->ops.user_ctx, DC_MOTOR_COAST);
        return;
    }

    mode = (speed > 0) ? DC_MOTOR_FORWARD : DC_MOTOR_REVERSE;
    if (motor->cfg.inverted != 0u)
    {
        mode = (mode == DC_MOTOR_FORWARD) ? DC_MOTOR_REVERSE : DC_MOTOR_FORWARD;
    }

    motor->ops.set_mode(motor->ops.user_ctx, mode);
    motor->ops.set_duty(motor->ops.user_ctx, abs_speed(speed));
}

void DcMotor_Coast(DcMotor *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->speed = 0;

    if (motor->ops.set_duty != NULL)
    {
        motor->ops.set_duty(motor->ops.user_ctx, 0u);
    }

    if (motor->ops.set_mode != NULL)
    {
        motor->ops.set_mode(motor->ops.user_ctx, DC_MOTOR_COAST);
    }
}

void DcMotor_Brake(DcMotor *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->speed = 0;

    if (motor->ops.set_duty != NULL)
    {
        motor->ops.set_duty(motor->ops.user_ctx, DC_MOTOR_MAX_SPEED);
    }

    if (motor->ops.set_mode != NULL)
    {
        motor->ops.set_mode(motor->ops.user_ctx, DC_MOTOR_BRAKE);
    }
}

int16_t DcMotor_GetSpeed(const DcMotor *motor)
{
    return (motor != NULL) ? motor->speed : 0;
}
