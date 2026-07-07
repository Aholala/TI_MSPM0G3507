/**
 * @file app_chassis.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "app_chassis.h"

#include "bsp_encoder.h"
#include "bsp_tb6612.h"
#include "board_config.h"
#include "module_motor.h"
#include "main.h"
#include "module_pid.h"

static DcMotor g_left_motor;
static DcMotor g_right_motor;
PidController g_app_chassis_left_speed_pid;
PidController g_app_chassis_right_speed_pid;
static AppChassis_Snapshot g_chassis_snapshot;
AppChassis_Snapshot g_debug_chassis_snapshot;
static int16_t g_left_encoder_delta;
static int16_t g_right_encoder_delta;
static int16_t g_left_target_speed;
static int16_t g_right_target_speed;
static int16_t g_left_control_output;
static int16_t g_right_control_output;

static int16_t clamp_chassis_speed(int16_t speed)
{
    if (speed > APP_CHASSIS_MAX_SPEED)
    {
        return APP_CHASSIS_MAX_SPEED;
    }

    if (speed < -APP_CHASSIS_MAX_SPEED)
    {
        return -APP_CHASSIS_MAX_SPEED;
    }

    return speed;
}

static uint32_t enter_snapshot_lock(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    return primask;
}

static void leave_snapshot_lock(uint32_t primask)
{
    if (primask == 0u)
    {
        __enable_irq();
    }
}

static void update_snapshot(void)
{
    uint32_t primask = enter_snapshot_lock();

    g_chassis_snapshot.left_speed = DcMotor_GetSpeed(&g_left_motor);
    g_chassis_snapshot.right_speed = DcMotor_GetSpeed(&g_right_motor);
    g_chassis_snapshot.left_target_speed = g_left_target_speed;
    g_chassis_snapshot.right_target_speed = g_right_target_speed;
    g_chassis_snapshot.left_encoder_delta = g_left_encoder_delta;
    g_chassis_snapshot.right_encoder_delta = g_right_encoder_delta;
    g_chassis_snapshot.left_encoder_total = BspEncoder_GetTotal(BSP_ENCODER_LEFT);
    g_chassis_snapshot.right_encoder_total = BspEncoder_GetTotal(BSP_ENCODER_RIGHT);
    g_chassis_snapshot.left_encoder_mrev = BspEncoder_GetTotalMilliRevolutions(BSP_ENCODER_LEFT);
    g_chassis_snapshot.right_encoder_mrev = BspEncoder_GetTotalMilliRevolutions(BSP_ENCODER_RIGHT);
    g_chassis_snapshot.left_encoder_mm = BspEncoder_GetTotalMillimeters(BSP_ENCODER_LEFT);
    g_chassis_snapshot.right_encoder_mm = BspEncoder_GetTotalMillimeters(BSP_ENCODER_RIGHT);
    g_chassis_snapshot.left_control_output = g_left_control_output;
    g_chassis_snapshot.right_control_output = g_right_control_output;
    g_debug_chassis_snapshot = g_chassis_snapshot;

    leave_snapshot_lock(primask);
}

void AppChassis_Init(void)
{
    DcMotor_Ops left_ops;
    DcMotor_Ops right_ops;
    DcMotor_Config left_cfg = {
        .inverted = 0u,
    };
    DcMotor_Config right_cfg = {
        .inverted = 0u,
    };
    PidController_Config speed_pid_cfg = {
        .kp = BOARD_CHASSIS_SPEED_PID_KP,
        .ki = BOARD_CHASSIS_SPEED_PID_KI,
        .kd = BOARD_CHASSIS_SPEED_PID_KD,
        .scale = BOARD_CHASSIS_SPEED_PID_SCALE,
        .integral_min = BOARD_CHASSIS_SPEED_PID_INTEGRAL_MIN,
        .integral_max = BOARD_CHASSIS_SPEED_PID_INTEGRAL_MAX,
        .output_min = BOARD_CHASSIS_SPEED_PID_OUTPUT_MIN,
        .output_max = BOARD_CHASSIS_SPEED_PID_OUTPUT_MAX,
    };

    BspTb6612_Init();
    BspEncoder_Init();
    BspTb6612_BindMotor(BSP_TB6612_MOTOR_LEFT, &left_ops);
    BspTb6612_BindMotor(BSP_TB6612_MOTOR_RIGHT, &right_ops);
    PidController_Init(&g_app_chassis_left_speed_pid, &speed_pid_cfg);
    PidController_Init(&g_app_chassis_right_speed_pid, &speed_pid_cfg);

    DcMotor_Init(&g_left_motor, &left_ops, &left_cfg);
    DcMotor_Init(&g_right_motor, &right_ops, &right_cfg);
    g_left_target_speed = 0;
    g_right_target_speed = 0;
    g_left_control_output = 0;
    g_right_control_output = 0;
    update_snapshot();
}

void AppChassis_UpdateEncoder(void)
{
    g_left_encoder_delta = BspEncoder_ReadDelta(BSP_ENCODER_LEFT);
    g_right_encoder_delta = BspEncoder_ReadDelta(BSP_ENCODER_RIGHT);
    update_snapshot();
}

void AppChassis_SetSpeedPidConfig(const PidController_Config *left_cfg,
                                  const PidController_Config *right_cfg)
{
    uint32_t primask = enter_snapshot_lock();

    if (left_cfg != NULL)
    {
        PidController_SetConfig(&g_app_chassis_left_speed_pid, left_cfg);
        PidController_Reset(&g_app_chassis_left_speed_pid);
    }

    if (right_cfg != NULL)
    {
        PidController_SetConfig(&g_app_chassis_right_speed_pid, right_cfg);
        PidController_Reset(&g_app_chassis_right_speed_pid);
    }

    leave_snapshot_lock(primask);
}

PidController *AppChassis_GetLeftSpeedPid(void)
{
    return &g_app_chassis_left_speed_pid;
}

PidController *AppChassis_GetRightSpeedPid(void)
{
    return &g_app_chassis_right_speed_pid;
}

void AppChassis_SetTargetSpeed(int16_t left_target, int16_t right_target)
{
    uint32_t primask = enter_snapshot_lock();

    g_left_target_speed = clamp_chassis_speed(left_target);
    g_right_target_speed = clamp_chassis_speed(right_target);

    leave_snapshot_lock(primask);
    update_snapshot();
}

void AppChassis_SpeedControlRun(void)
{
    int16_t left_target;
    int16_t right_target;
    uint32_t primask = enter_snapshot_lock();

    left_target = g_left_target_speed;
    right_target = g_right_target_speed;
    leave_snapshot_lock(primask);

    g_left_control_output = clamp_chassis_speed((int16_t)PidController_Run(
        &g_app_chassis_left_speed_pid,
        left_target,
        g_left_encoder_delta));
    g_right_control_output = clamp_chassis_speed((int16_t)PidController_Run(
        &g_app_chassis_right_speed_pid,
        right_target,
        g_right_encoder_delta));
    AppChassis_SetMotorSpeed(g_left_control_output, g_right_control_output);
}

void AppChassis_SetMotorSpeed(int16_t left_speed, int16_t right_speed)
{
    left_speed = clamp_chassis_speed(left_speed);
    right_speed = clamp_chassis_speed(right_speed);

    DcMotor_SetSpeed(&g_left_motor, left_speed);
    DcMotor_SetSpeed(&g_right_motor, right_speed);
    update_snapshot();
}

void AppChassis_SetVelocity(int16_t forward, int16_t turn)
{
    int32_t left = (int32_t)forward - turn;
    int32_t right = (int32_t)forward + turn;

    AppChassis_SetMotorSpeed(clamp_chassis_speed((int16_t)left),
                             clamp_chassis_speed((int16_t)right));
}

void AppChassis_Stop(void)
{
    AppChassis_SetTargetSpeed(0, 0);
    DcMotor_Coast(&g_left_motor);
    DcMotor_Coast(&g_right_motor);
    g_left_control_output = 0;
    g_right_control_output = 0;
    update_snapshot();
}

void AppChassis_Brake(void)
{
    AppChassis_SetTargetSpeed(0, 0);
    DcMotor_Brake(&g_left_motor);
    DcMotor_Brake(&g_right_motor);
    g_left_control_output = 0;
    g_right_control_output = 0;
    update_snapshot();
}

AppChassis_Snapshot AppChassis_GetSnapshot(void)
{
    AppChassis_Snapshot snapshot;
    uint32_t primask = enter_snapshot_lock();

    snapshot = g_chassis_snapshot;
    leave_snapshot_lock(primask);

    return snapshot;
}
