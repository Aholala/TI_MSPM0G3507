#include "app_freertos.h"

#include "FreeRTOS.h"
#include "task.h"

static void App_MainTask(void *argument);

void App_FreeRTOS_CreateTasks(void)
{
    BaseType_t result;

    result = xTaskCreate(App_MainTask,
                         "app",
                         configMINIMAL_STACK_SIZE,
                         NULL,
                         tskIDLE_PRIORITY + 1,
                         NULL);
    configASSERT(result == pdPASS);
}

static void App_MainTask(void *argument)
{
    (void) argument;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
