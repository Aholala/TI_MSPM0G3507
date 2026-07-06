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
