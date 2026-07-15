#include "bsp_tb6612.h"

#include <stddef.h>

#include "board_config.h"
#include "ti_msp_dl_config.h"

typedef struct
{
    BspTb6612_MotorId id;
    GPIO_Regs *in1_port;
    uint32_t in1_pin;
    GPIO_Regs *in2_port;
    uint32_t in2_pin;
    DcMotor_Mode current_mode;
    uint8_t mode_initialized;
} BspTb6612_MotorHw;

static BspTb6612_MotorHw motor_hw[BSP_TB6612_MOTOR_COUNT] = {
    {
        BSP_TB6612_MOTOR_LEFT,
        BOARD_TB6612_LEFT_IN1_PORT,
        BOARD_TB6612_LEFT_IN1_PIN,
        BOARD_TB6612_LEFT_IN2_PORT,
        BOARD_TB6612_LEFT_IN2_PIN,
    },
    {
        BSP_TB6612_MOTOR_RIGHT,
        BOARD_TB6612_RIGHT_IN1_PORT,
        BOARD_TB6612_RIGHT_IN1_PIN,
        BOARD_TB6612_RIGHT_IN2_PORT,
        BOARD_TB6612_RIGHT_IN2_PIN,
    },
};

static void write_pin(GPIO_Regs *port, uint32_t pin, uint8_t high)
{
    if (high != 0u)
    {
        DL_GPIO_setPins(port, pin);
    }
    else
    {
        DL_GPIO_clearPins(port, pin);
    }
}

static void set_compare(BspTb6612_MotorId motor_id, uint32_t compare)
{
    if (motor_id == BSP_TB6612_MOTOR_LEFT)
    {
        DL_TimerG_setCaptureCompareValue(MOTOR_PWMA_INST,
                                         compare,
                                         GPIO_MOTOR_PWMA_C0_IDX);
    }
    else
    {
        DL_TimerA_setCaptureCompareValue(MOTOR_PWMB_INST,
                                         compare,
                                         GPIO_MOTOR_PWMB_C0_IDX);
    }
}

static void set_duty(void *user_ctx, uint16_t duty)
{
    const BspTb6612_MotorHw *hw = (const BspTb6612_MotorHw *)user_ctx;
    uint32_t compare;

    if (hw == NULL)
    {
        return;
    }
    if (duty > BOARD_TB6612_PWM_MAX_DUTY)
    {
        duty = BOARD_TB6612_PWM_MAX_DUTY;
    }

    /* SysConfig PWM output is active while counter >= compare. */
    compare = BOARD_TB6612_PWM_PERIOD_COUNTS -
              (((uint32_t)duty * BOARD_TB6612_PWM_PERIOD_COUNTS) /
               BOARD_TB6612_PWM_MAX_DUTY);
    set_compare(hw->id, compare);
}

static void set_mode(void *user_ctx, DcMotor_Mode mode)
{
    BspTb6612_MotorHw *hw = (BspTb6612_MotorHw *)user_ctx;
    uint8_t in1 = 0u;
    uint8_t in2 = 0u;

    if (hw == NULL)
    {
        return;
    }

    /* The speed loop calls this every 10 ms. Do not blank PWM when the bridge
     * direction is unchanged, otherwise the motor is interrupted at 100 Hz. */
    if ((hw->mode_initialized != 0u) && (hw->current_mode == mode))
    {
        return;
    }

    /* Remove bridge drive before changing its direction inputs. */
    set_duty((void *)hw, 0u);

    switch (mode)
    {
    case DC_MOTOR_FORWARD:
        in1 = 1u;
        break;

    case DC_MOTOR_REVERSE:
        in2 = 1u;
        break;

    case DC_MOTOR_BRAKE:
        in1 = 1u;
        in2 = 1u;
        break;

    case DC_MOTOR_COAST:
    default:
        break;
    }

    write_pin(hw->in1_port, hw->in1_pin, in1);
    write_pin(hw->in2_port, hw->in2_pin, in2);
    hw->current_mode = mode;
    hw->mode_initialized = 1u;

    if (mode == DC_MOTOR_BRAKE)
    {
        set_duty((void *)hw, BOARD_TB6612_PWM_MAX_DUTY);
    }
}

void BspTb6612_Init(void)
{
    motor_hw[BSP_TB6612_MOTOR_LEFT].mode_initialized = 0u;
    motor_hw[BSP_TB6612_MOTOR_RIGHT].mode_initialized = 0u;
    set_duty((void *)&motor_hw[BSP_TB6612_MOTOR_LEFT], 0u);
    set_duty((void *)&motor_hw[BSP_TB6612_MOTOR_RIGHT], 0u);
    set_mode((void *)&motor_hw[BSP_TB6612_MOTOR_LEFT], DC_MOTOR_COAST);
    set_mode((void *)&motor_hw[BSP_TB6612_MOTOR_RIGHT], DC_MOTOR_COAST);

    DL_TimerG_startCounter(MOTOR_PWMA_INST);
    DL_TimerA_startCounter(MOTOR_PWMB_INST);
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
