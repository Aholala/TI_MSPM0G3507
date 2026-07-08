/**
 * @file chassis_task.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "chassis_task.h"

#include "app_chassis.h"
#include "FreeRTOS.h"
#include "task.h"

#define CHASSIS_TASK_PERIOD_MS 10U

void StartchassisTask(void *argument)
{
    TickType_t wake_tick = xTaskGetTickCount();

    (void)argument;

    for (;;)
    {
        AppChassis_UpdateEncoder();
        AppChassis_SpeedControlRun();
        vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CHASSIS_TASK_PERIOD_MS));
    }
}
