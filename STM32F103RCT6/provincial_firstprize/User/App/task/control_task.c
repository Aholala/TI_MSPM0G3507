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
#include "lib_rate_filter.h"

#define CONTROL_TASK_PERIOD_MS BOARD_CONTROL_TASK_PERIOD_MS
#define CONTROL_TASK_LINE_ERROR_DIV 300
#define CONTROL_TASK_DEFAULT_BASE_SPEED ((int16_t)BOARD_CHASSIS_DEFAULT_SPEED_DELTA)
#define CONTROL_TASK_LINE_FILTER_DIV 2u
#define CONTROL_TASK_LINE_ERROR_DEADBAND 300
#define CONTROL_TASK_TURN_SLEW_STEP 2

int16_t g_control_task_base_speed;
int16_t g_control_task_turn;

void StartcontrolTask(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();
    LowPassFilter line_error_filter;

    (void)argument;
    LowPassFilter_Init(&line_error_filter, CONTROL_TASK_LINE_FILTER_DIV);

    for (;;)
    {
        AppLineFollow_Snapshot line_snapshot;
        int32_t filtered_error;
        int16_t target_turn;
        int16_t base_speed;

        if (AppRaceConfig_IsRunning() == 0u)
        {
            g_control_task_turn = 0;
            LowPassFilter_Reset(&line_error_filter);
            AppChassis_Stop();
            wake_tick += CONTROL_TASK_PERIOD_MS;
            osDelayUntil(wake_tick);
            continue;
        }

        line_snapshot = AppLineFollow_GetSnapshot();
        if (line_snapshot.status != LINE_TRACKER_OK)
        {
            /*
             * No line (LOST) and all sensors active (FULL) are ambiguous.
             * Do not feed either condition into the speed loop: its integral
             * term would otherwise keep driving the motors toward saturation.
             */
            g_control_task_turn = 0;
            LowPassFilter_Reset(&line_error_filter);
            AppChassis_Stop();
            wake_tick += CONTROL_TASK_PERIOD_MS;
            osDelayUntil(wake_tick);
            continue;
        }

        base_speed = (g_control_task_base_speed != 0) ? g_control_task_base_speed : CONTROL_TASK_DEFAULT_BASE_SPEED;
        filtered_error = LowPassFilter_Update(&line_error_filter, line_snapshot.error);
        if ((filtered_error >= -CONTROL_TASK_LINE_ERROR_DEADBAND) &&
            (filtered_error <= CONTROL_TASK_LINE_ERROR_DEADBAND))
        {
            target_turn = 0;
        }
        else
        {
            target_turn = (int16_t)(filtered_error / CONTROL_TASK_LINE_ERROR_DIV);
        }

        if (target_turn > (int16_t)(g_control_task_turn + CONTROL_TASK_TURN_SLEW_STEP))
        {
            ++g_control_task_turn;
        }
        else if (target_turn < (int16_t)(g_control_task_turn - CONTROL_TASK_TURN_SLEW_STEP))
        {
            --g_control_task_turn;
        }
        else
        {
            g_control_task_turn = target_turn;
        }

        /* Keep line following from commanding one wheel in reverse. */
        if (g_control_task_turn > base_speed)
        {
            g_control_task_turn = base_speed;
        }
        else if (g_control_task_turn < -base_speed)
        {
            g_control_task_turn = (int16_t)-base_speed;
        }

        AppChassis_SetTargetSpeed((int16_t)(base_speed + g_control_task_turn),
                                  (int16_t)(base_speed - g_control_task_turn));

        wake_tick += CONTROL_TASK_PERIOD_MS;
        osDelayUntil(wake_tick);
    }
}
