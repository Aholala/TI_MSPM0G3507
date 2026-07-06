#include "control_task.h"

#include "app_chassis.h"
#include "app_line_follow.h"
#include "cmsis_os.h"

#define CONTROL_TASK_PERIOD_MS 10U
#define CONTROL_TASK_LINE_ERROR_DIV 20

int16_t g_control_task_base_speed;
int16_t g_control_task_turn;

void StartcontrolTask(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();

    (void)argument;

    for (;;)
    {
        int16_t line_error = AppLineFollow_GetError();

        g_control_task_turn = (int16_t)(line_error / CONTROL_TASK_LINE_ERROR_DIV);
        AppChassis_SetTargetSpeed((int16_t)(g_control_task_base_speed - g_control_task_turn),
                                  (int16_t)(g_control_task_base_speed + g_control_task_turn));

        wake_tick += CONTROL_TASK_PERIOD_MS;
        osDelayUntil(wake_tick);
    }
}
