/**
 * @file line_follow_task.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "line_follow_task.h"

#include "app_line_follow.h"
#include "cmsis_os.h"

#define LINE_FOLLOW_TASK_PERIOD_MS 5U

void StartlinefollowTask(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();

    (void)argument;

    for (;;)
    {
        (void)AppLineFollow_RunOnce();
        wake_tick += LINE_FOLLOW_TASK_PERIOD_MS;
        osDelayUntil(wake_tick);
    }
}
