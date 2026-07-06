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
