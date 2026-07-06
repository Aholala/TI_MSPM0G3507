#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include <stdint.h>

#include "module_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_CHASSIS_MAX_SPEED 1000

extern PidController g_app_chassis_left_speed_pid;
extern PidController g_app_chassis_right_speed_pid;

typedef struct
{
    int16_t left_speed;
    int16_t right_speed;
    int16_t left_target_speed;
    int16_t right_target_speed;
    int16_t left_encoder_delta;
    int16_t right_encoder_delta;
    int32_t left_encoder_total;
    int32_t right_encoder_total;
    int16_t left_control_output;
    int16_t right_control_output;
} AppChassis_Snapshot;

void AppChassis_Init(void);
void AppChassis_UpdateEncoder(void);
void AppChassis_SetSpeedPidConfig(const PidController_Config *left_cfg,
                                  const PidController_Config *right_cfg);
PidController *AppChassis_GetLeftSpeedPid(void);
PidController *AppChassis_GetRightSpeedPid(void);
void AppChassis_SetTargetSpeed(int16_t left_target, int16_t right_target);
void AppChassis_SpeedControlRun(void);
void AppChassis_SetMotorSpeed(int16_t left_speed, int16_t right_speed);
void AppChassis_SetVelocity(int16_t forward, int16_t turn);
void AppChassis_Stop(void);
void AppChassis_Brake(void);
AppChassis_Snapshot AppChassis_GetSnapshot(void);

#ifdef __cplusplus
}
#endif

#endif
