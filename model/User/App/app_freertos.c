#include "app_freertos.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_chassis.h"
#include "app_line_follow.h"
#include "app_race_config.h"
#include "task/chassis_task.h"
#include "task/control_task.h"
#include "task/line_follow_task.h"

#define APP_TASK_STACK_WORDS 256u
#define APP_CONTROL_TASK_STACK_WORDS 768u

static void race_config_task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;
    for (;;)
    {
        uint32_t now_ms = (uint32_t)xTaskGetTickCount();

        AppRaceConfig_RunOnce(now_ms);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_RACE_CONFIG_PERIOD_MS));
    }
}

/* Keep key sampling independent from the relatively slow software-I2C OLED
 * refresh so a short press cannot disappear while the display is updating. */
static void key_scan_task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;
    for (;;)
    {
        AppRaceConfig_KeyTimer1ms((uint32_t)xTaskGetTickCount());
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1u));
    }
}

static void oled_task(void *argument)
{
    (void)argument;
    for (;;)
    {
        AppRaceConfig_DisplayRunOnce((uint32_t)xTaskGetTickCount());
        vTaskDelay(pdMS_TO_TICKS(1u));
    }
}

static void create_task(TaskFunction_t entry,
                        const char *name,
                        configSTACK_DEPTH_TYPE stack_words,
                        UBaseType_t priority)
{
    BaseType_t result = xTaskCreate(entry,
                                    name,
                                    stack_words,
                                    NULL,
                                    priority,
                                    NULL);
    configASSERT(result == pdPASS);
}

void App_FreeRTOS_CreateTasks(void)
{
    AppChassis_Init();
    AppLineFollow_Init();
    AppRaceConfig_Init(0u);

    create_task(StartchassisTask, "chassis", APP_TASK_STACK_WORDS, 3u);
    create_task(key_scan_task, "key_scan", configMINIMAL_STACK_SIZE, 3u);
    create_task(StartlinefollowTask, "line", APP_TASK_STACK_WORDS, 2u);
    create_task(StartcontrolTask, "control", APP_CONTROL_TASK_STACK_WORDS, 2u);
    create_task(race_config_task, "race", 192u, 1u);
    create_task(oled_task, "oled", APP_TASK_STACK_WORDS, 1u);
}
