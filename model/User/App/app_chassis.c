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
#include "lib_rate_filter.h"
#include "module_motor.h"
#include "ti_msp_dl_config.h"

static DcMotor g_left_motor;
static DcMotor g_right_motor;
PidController g_app_chassis_left_speed_pid;
PidController g_app_chassis_right_speed_pid;
static RateFilter g_left_rpm_filter;
static RateFilter g_right_rpm_filter;
static AppChassis_Snapshot g_chassis_snapshot;
AppChassis_Snapshot g_debug_chassis_snapshot;
volatile int32_t g_left_speed_pid_kp_x100 = BOARD_CHASSIS_LEFT_SPEED_PID_KP;
volatile int32_t g_left_speed_pid_ki_x100 = BOARD_CHASSIS_LEFT_SPEED_PID_KI;
volatile int32_t g_left_speed_pid_kd_x100 = BOARD_CHASSIS_LEFT_SPEED_PID_KD;
volatile int32_t g_right_speed_pid_kp_x100 = BOARD_CHASSIS_RIGHT_SPEED_PID_KP;
volatile int32_t g_right_speed_pid_ki_x100 = BOARD_CHASSIS_RIGHT_SPEED_PID_KI;
volatile int32_t g_right_speed_pid_kd_x100 = BOARD_CHASSIS_RIGHT_SPEED_PID_KD;
volatile int16_t g_left_speed_feedforward_at_rated =
    BOARD_CHASSIS_LEFT_SPEED_FEEDFORWARD_AT_RATED;
volatile int16_t g_right_speed_feedforward_at_rated =
    BOARD_CHASSIS_RIGHT_SPEED_FEEDFORWARD_AT_RATED;
static int32_t g_applied_left_pid_kp_x100 = BOARD_CHASSIS_LEFT_SPEED_PID_KP;
static int32_t g_applied_left_pid_ki_x100 = BOARD_CHASSIS_LEFT_SPEED_PID_KI;
static int32_t g_applied_left_pid_kd_x100 = BOARD_CHASSIS_LEFT_SPEED_PID_KD;
static int32_t g_applied_right_pid_kp_x100 = BOARD_CHASSIS_RIGHT_SPEED_PID_KP;
static int32_t g_applied_right_pid_ki_x100 = BOARD_CHASSIS_RIGHT_SPEED_PID_KI;
static int32_t g_applied_right_pid_kd_x100 = BOARD_CHASSIS_RIGHT_SPEED_PID_KD;
static int16_t g_left_encoder_delta;
static int16_t g_right_encoder_delta;
static int16_t g_left_target_speed;
static int16_t g_right_target_speed;
static int16_t g_left_control_output;
static int16_t g_right_control_output;
static uint8_t g_brake_active;

#define APP_CHASSIS_RPM_WINDOW_SAMPLES 5u
#define APP_CHASSIS_RPM_FILTER_DIV 8u
#define APP_CHASSIS_TARGET_SLEW_STEP 4

static void apply_runtime_pid_tuning(void)
{
    if ((g_left_speed_pid_kp_x100 != g_applied_left_pid_kp_x100) ||
        (g_left_speed_pid_ki_x100 != g_applied_left_pid_ki_x100) ||
        (g_left_speed_pid_kd_x100 != g_applied_left_pid_kd_x100))
    {
        PidController_Config left_cfg = g_app_chassis_left_speed_pid.cfg;

        left_cfg.kp = g_left_speed_pid_kp_x100;
        left_cfg.ki = g_left_speed_pid_ki_x100;
        left_cfg.kd = g_left_speed_pid_kd_x100;
        AppChassis_SetSpeedPidConfig(&left_cfg, NULL);
        g_applied_left_pid_kp_x100 = g_left_speed_pid_kp_x100;
        g_applied_left_pid_ki_x100 = g_left_speed_pid_ki_x100;
        g_applied_left_pid_kd_x100 = g_left_speed_pid_kd_x100;
    }

    if ((g_right_speed_pid_kp_x100 != g_applied_right_pid_kp_x100) ||
        (g_right_speed_pid_ki_x100 != g_applied_right_pid_ki_x100) ||
        (g_right_speed_pid_kd_x100 != g_applied_right_pid_kd_x100))
    {
        PidController_Config right_cfg = g_app_chassis_right_speed_pid.cfg;

        right_cfg.kp = g_right_speed_pid_kp_x100;
        right_cfg.ki = g_right_speed_pid_ki_x100;
        right_cfg.kd = g_right_speed_pid_kd_x100;
        AppChassis_SetSpeedPidConfig(NULL, &right_cfg);
        g_applied_right_pid_kp_x100 = g_right_speed_pid_kp_x100;
        g_applied_right_pid_ki_x100 = g_right_speed_pid_ki_x100;
        g_applied_right_pid_kd_x100 = g_right_speed_pid_kd_x100;
    }
}

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

static int16_t clamp_target_speed(int16_t speed)
{
    const int16_t rated_delta = (int16_t)BOARD_CHASSIS_RATED_SPEED_DELTA;

    if (speed > rated_delta)
    {
        return rated_delta;
    }

    if (speed < -rated_delta)
    {
        return (int16_t)-rated_delta;
    }

    return speed;
}

static int16_t speed_feedforward(int16_t target, int16_t configured_feedforward)
{
    int32_t feedforward_at_rated = configured_feedforward;
    int32_t feedforward;

    if ((target == 0) || (BOARD_CHASSIS_RATED_SPEED_DELTA == 0u))
    {
        return 0;
    }
    if (feedforward_at_rated < 0)
    {
        feedforward_at_rated = 0;
    }
    else if (feedforward_at_rated > APP_CHASSIS_MAX_SPEED)
    {
        feedforward_at_rated = APP_CHASSIS_MAX_SPEED;
    }

    feedforward = ((int32_t)target * feedforward_at_rated) /
                  (int32_t)BOARD_CHASSIS_RATED_SPEED_DELTA;
    return clamp_chassis_speed((int16_t)feedforward);
}

static int16_t slew_target_speed(int16_t current, int16_t target)
{
    if (target > (int16_t)(current + APP_CHASSIS_TARGET_SLEW_STEP))
    {
        return (int16_t)(current + APP_CHASSIS_TARGET_SLEW_STEP);
    }
    if (target < (int16_t)(current - APP_CHASSIS_TARGET_SLEW_STEP))
    {
        return (int16_t)(current - APP_CHASSIS_TARGET_SLEW_STEP);
    }
    return target;
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
    g_chassis_snapshot.left_target_rpm =
        RateFilter_DeltaToRate(&g_left_rpm_filter, g_left_target_speed);
    g_chassis_snapshot.right_target_rpm =
        RateFilter_DeltaToRate(&g_right_rpm_filter, g_right_target_speed);
    g_chassis_snapshot.left_encoder_delta = g_left_encoder_delta;
    g_chassis_snapshot.right_encoder_delta = g_right_encoder_delta;
    g_chassis_snapshot.left_actual_rpm = RateFilter_GetInstantRate(&g_left_rpm_filter);
    g_chassis_snapshot.right_actual_rpm = RateFilter_GetInstantRate(&g_right_rpm_filter);
    g_chassis_snapshot.left_actual_rpm_filtered =
        RateFilter_GetFilteredRate(&g_left_rpm_filter);
    g_chassis_snapshot.right_actual_rpm_filtered =
        RateFilter_GetFilteredRate(&g_right_rpm_filter);
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
        .inverted = BOARD_TB6612_LEFT_MOTOR_INVERTED,
    };
    DcMotor_Config right_cfg = {
        .inverted = BOARD_TB6612_RIGHT_MOTOR_INVERTED,
    };
    PidController_Config left_speed_pid_cfg = {
        .kp = BOARD_CHASSIS_LEFT_SPEED_PID_KP,
        .ki = BOARD_CHASSIS_LEFT_SPEED_PID_KI,
        .kd = BOARD_CHASSIS_LEFT_SPEED_PID_KD,
        .scale = BOARD_CHASSIS_SPEED_PID_SCALE,
        .integral_min = BOARD_CHASSIS_SPEED_PID_INTEGRAL_MIN,
        .integral_max = BOARD_CHASSIS_SPEED_PID_INTEGRAL_MAX,
        .output_min = BOARD_CHASSIS_SPEED_PID_OUTPUT_MIN,
        .output_max = BOARD_CHASSIS_SPEED_PID_OUTPUT_MAX,
    };
    PidController_Config right_speed_pid_cfg = {
        .kp = BOARD_CHASSIS_RIGHT_SPEED_PID_KP,
        .ki = BOARD_CHASSIS_RIGHT_SPEED_PID_KI,
        .kd = BOARD_CHASSIS_RIGHT_SPEED_PID_KD,
        .scale = BOARD_CHASSIS_SPEED_PID_SCALE,
        .integral_min = BOARD_CHASSIS_SPEED_PID_INTEGRAL_MIN,
        .integral_max = BOARD_CHASSIS_SPEED_PID_INTEGRAL_MAX,
        .output_min = BOARD_CHASSIS_SPEED_PID_OUTPUT_MIN,
        .output_max = BOARD_CHASSIS_SPEED_PID_OUTPUT_MAX,
    };
    RateFilter_Config rpm_filter_cfg = {
        .units_per_cycle = BOARD_ENCODER_OUTPUT_PULSES_PER_REV,
        .sample_period_ms = BOARD_CONTROL_TASK_PERIOD_MS,
        .window_samples = APP_CHASSIS_RPM_WINDOW_SAMPLES,
        .filter_div = APP_CHASSIS_RPM_FILTER_DIV,
    };

    BspTb6612_Init();
    BspEncoder_Init();
    BspTb6612_BindMotor(BSP_TB6612_MOTOR_LEFT, &left_ops);
    BspTb6612_BindMotor(BSP_TB6612_MOTOR_RIGHT, &right_ops);
    PidController_Init(&g_app_chassis_left_speed_pid, &left_speed_pid_cfg);
    PidController_Init(&g_app_chassis_right_speed_pid, &right_speed_pid_cfg);
    RateFilter_Init(&g_left_rpm_filter, &rpm_filter_cfg);
    RateFilter_Init(&g_right_rpm_filter, &rpm_filter_cfg);

    DcMotor_Init(&g_left_motor, &left_ops, &left_cfg);
    DcMotor_Init(&g_right_motor, &right_ops, &right_cfg);
    g_left_target_speed = 0;
    g_right_target_speed = 0;
    g_left_control_output = 0;
    g_right_control_output = 0;
    g_brake_active = 0u;
    update_snapshot();
}

void AppChassis_UpdateEncoder(void)
{
    g_left_encoder_delta = BspEncoder_ReadDelta(BSP_ENCODER_LEFT);
    g_right_encoder_delta = BspEncoder_ReadDelta(BSP_ENCODER_RIGHT);
    RateFilter_Update(&g_left_rpm_filter, g_left_encoder_delta);
    RateFilter_Update(&g_right_rpm_filter, g_right_encoder_delta);
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

    g_left_target_speed = clamp_target_speed(left_target);
    g_right_target_speed = clamp_target_speed(right_target);
    if ((g_left_target_speed != 0) || (g_right_target_speed != 0))
    {
        g_brake_active = 0u;
    }

    leave_snapshot_lock(primask);
    update_snapshot();
}

void AppChassis_SetTargetVelocity(int16_t forward, int16_t turn)
{
    int32_t left = (int32_t)forward + turn;
    int32_t right = (int32_t)forward - turn;
    int16_t left_target;
    int16_t right_target;

    if (left > INT16_MAX)
    {
        left = INT16_MAX;
    }
    else if (left < INT16_MIN)
    {
        left = INT16_MIN;
    }
    if (right > INT16_MAX)
    {
        right = INT16_MAX;
    }
    else if (right < INT16_MIN)
    {
        right = INT16_MIN;
    }

    left_target = clamp_target_speed((int16_t)left);
    right_target = clamp_target_speed((int16_t)right);
    left_target = slew_target_speed(g_left_target_speed, left_target);
    right_target = slew_target_speed(g_right_target_speed, right_target);
    AppChassis_SetTargetSpeed(left_target, right_target);
}

void AppChassis_SpeedControlRun(void)
{
    int16_t left_target;
    int16_t right_target;
    uint32_t primask;

    apply_runtime_pid_tuning();
    primask = enter_snapshot_lock();

    left_target = g_left_target_speed;
    right_target = g_right_target_speed;
    leave_snapshot_lock(primask);

    if ((left_target == 0) && (right_target == 0))
    {
        PidController_Reset(&g_app_chassis_left_speed_pid);
        PidController_Reset(&g_app_chassis_right_speed_pid);
        RateFilter_Reset(&g_left_rpm_filter);
        RateFilter_Reset(&g_right_rpm_filter);
        g_left_control_output = 0;
        g_right_control_output = 0;
        if (g_brake_active != 0u)
        {
            DcMotor_Brake(&g_left_motor);
            DcMotor_Brake(&g_right_motor);
        }
        else
        {
            AppChassis_SetMotorSpeed(0, 0);
        }
        return;
    }

    if (left_target == 0)
    {
        PidController_Reset(&g_app_chassis_left_speed_pid);
        g_left_control_output = 0;
    }
    else
    {
        int32_t correction = PidController_Run(&g_app_chassis_left_speed_pid,
                                               left_target,
                                               g_left_encoder_delta);
        g_left_control_output = clamp_chassis_speed(
            (int16_t)(speed_feedforward(left_target,
                                       g_left_speed_feedforward_at_rated) +
                      correction));
    }

    if (right_target == 0)
    {
        PidController_Reset(&g_app_chassis_right_speed_pid);
        g_right_control_output = 0;
    }
    else
    {
        int32_t correction = PidController_Run(&g_app_chassis_right_speed_pid,
                                               right_target,
                                               g_right_encoder_delta);
        g_right_control_output = clamp_chassis_speed(
            (int16_t)(speed_feedforward(right_target,
                                       g_right_speed_feedforward_at_rated) +
                      correction));
    }
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
    int32_t left = (int32_t)forward + turn;
    int32_t right = (int32_t)forward - turn;

    AppChassis_SetMotorSpeed(clamp_chassis_speed((int16_t)left),
                             clamp_chassis_speed((int16_t)right));
}

void AppChassis_Stop(void)
{
    AppChassis_SetTargetSpeed(0, 0);
    if (g_brake_active != 0u)
    {
        DcMotor_Brake(&g_left_motor);
        DcMotor_Brake(&g_right_motor);
    }
    else
    {
        DcMotor_Coast(&g_left_motor);
        DcMotor_Coast(&g_right_motor);
    }
    g_left_control_output = 0;
    g_right_control_output = 0;
    update_snapshot();
}

void AppChassis_Brake(void)
{
    AppChassis_SetTargetSpeed(0, 0);
    g_brake_active = 1u;
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
