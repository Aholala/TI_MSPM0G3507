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
#include "cmsis_os.h"

#define CHASSIS_TASK_PERIOD_MS 10U

void StartchassisTask(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();

    (void)argument;

    for (;;)
    {
        AppChassis_UpdateEncoder();
        AppChassis_SpeedControlRun();
        wake_tick += CHASSIS_TASK_PERIOD_MS;
        osDelayUntil(wake_tick);
    }
}
