/**
 * @file control_task.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "control_task.h"

#include "app_chassis.h"
#include "app_line_follow.h"
#include "app_race_config.h"
#include "board_config.h"
#include "cmsis_os.h"

#define CONTROL_TASK_PERIOD_MS BOARD_CONTROL_TASK_PERIOD_MS
#define CONTROL_TASK_LINE_ERROR_DIV 20
#define CONTROL_TASK_DEFAULT_BASE_SPEED ((int16_t)BOARD_CHASSIS_DEFAULT_SPEED_DELTA)

int16_t g_control_task_base_speed;
int16_t g_control_task_turn;

void StartcontrolTask(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();

    (void)argument;

    for (;;)
    {
        int16_t line_error;
        int16_t base_speed;

        if (AppRaceConfig_IsRunning() == 0u)
        {
            AppChassis_Stop();
            wake_tick += CONTROL_TASK_PERIOD_MS;
            osDelayUntil(wake_tick);
            continue;
        }

        line_error = AppLineFollow_GetError();
        base_speed = (g_control_task_base_speed != 0) ? g_control_task_base_speed : CONTROL_TASK_DEFAULT_BASE_SPEED;
        g_control_task_turn = (int16_t)(line_error / CONTROL_TASK_LINE_ERROR_DIV);
        AppChassis_SetTargetSpeed((int16_t)(base_speed - g_control_task_turn),
                                  (int16_t)(base_speed + g_control_task_turn));

        wake_tick += CONTROL_TASK_PERIOD_MS;
        osDelayUntil(wake_tick);
    }
}
