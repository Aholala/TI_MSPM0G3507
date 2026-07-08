/**
 * @file bsp_tb6612.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bsp_tb6612.h"

#include "tim.h"

typedef struct
{
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    uint32_t channel;
} BspTb6612_MotorHw;

static const BspTb6612_MotorHw motor_hw[BSP_TB6612_MOTOR_COUNT] = {
    {
        BOARD_TB6612_LEFT_IN1_GPIO_PORT,
        BOARD_TB6612_LEFT_IN1_GPIO_PIN,
        BOARD_TB6612_LEFT_IN2_GPIO_PORT,
        BOARD_TB6612_LEFT_IN2_GPIO_PIN,
        BOARD_TB6612_LEFT_PWM_CHANNEL,
    },
    {
        BOARD_TB6612_RIGHT_IN1_GPIO_PORT,
        BOARD_TB6612_RIGHT_IN1_GPIO_PIN,
        BOARD_TB6612_RIGHT_IN2_GPIO_PORT,
        BOARD_TB6612_RIGHT_IN2_GPIO_PIN,
        BOARD_TB6612_RIGHT_PWM_CHANNEL,
    },
};

static void tb6612_write_mode(const BspTb6612_MotorHw *hw, DcMotor_Mode mode)
{
    GPIO_PinState in1 = GPIO_PIN_RESET;
    GPIO_PinState in2 = GPIO_PIN_RESET;

    switch (mode)
    {
    case DC_MOTOR_FORWARD:
        in1 = GPIO_PIN_SET;
        in2 = GPIO_PIN_RESET;
        break;

    case DC_MOTOR_REVERSE:
        in1 = GPIO_PIN_RESET;
        in2 = GPIO_PIN_SET;
        break;

    case DC_MOTOR_BRAKE:
        in1 = GPIO_PIN_SET;
        in2 = GPIO_PIN_SET;
        break;

    case DC_MOTOR_COAST:
    default:
        in1 = GPIO_PIN_RESET;
        in2 = GPIO_PIN_RESET;
        break;
    }

    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, in1);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, in2);
}

static void set_mode(void *user_ctx, DcMotor_Mode mode)
{
    const BspTb6612_MotorHw *hw = (const BspTb6612_MotorHw *)user_ctx;

    if (hw == NULL)
    {
        return;
    }

    tb6612_write_mode(hw, mode);
}

static void set_duty(void *user_ctx, uint16_t duty)
{
    const BspTb6612_MotorHw *hw = (const BspTb6612_MotorHw *)user_ctx;
    uint32_t period;
    uint32_t compare;

    if (hw == NULL)
    {
        return;
    }

    if (duty > BOARD_TB6612_PWM_MAX_DUTY)
    {
        duty = BOARD_TB6612_PWM_MAX_DUTY;
    }

    period = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1u;
    compare = (period * duty) / BOARD_TB6612_PWM_MAX_DUTY;
    __HAL_TIM_SET_COMPARE(&htim3, hw->channel, compare);
}

void BspTb6612_Init(void)
{
    (void)HAL_TIM_PWM_Start(&htim3, BOARD_TB6612_LEFT_PWM_CHANNEL);
    (void)HAL_TIM_PWM_Start(&htim3, BOARD_TB6612_RIGHT_PWM_CHANNEL);
    set_duty((void *)&motor_hw[BSP_TB6612_MOTOR_LEFT], 0u);
    set_duty((void *)&motor_hw[BSP_TB6612_MOTOR_RIGHT], 0u);

    tb6612_write_mode(&motor_hw[BSP_TB6612_MOTOR_LEFT], DC_MOTOR_COAST);
    tb6612_write_mode(&motor_hw[BSP_TB6612_MOTOR_RIGHT], DC_MOTOR_COAST);
}

void BspTb6612_BindMotor(BspTb6612_MotorId motor_id, DcMotor_Ops *ops)
{
    if ((ops == NULL) || (motor_id >= BSP_TB6612_MOTOR_COUNT))
    {
        return;
    }

    ops->set_mode = set_mode;
    ops->set_duty = set_duty;
    ops->user_ctx = (void *)&motor_hw[motor_id];
}
