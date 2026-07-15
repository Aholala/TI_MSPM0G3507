/**
 * @file imu_task.c
 * @brief Periodically updates ICM45686 attitude estimation.
 */

#include "imu_task.h"

#include "app_icm45686.h"
#include "cmsis_os.h"

#define IMU_TASK_INIT_RETRY_MS 100U

void StartimuTask(void *argument)
{
	uint32_t wake_tick;
	int init_status;

	(void)argument;

	do {
		init_status = icm45686_app_init(NULL);
		if (init_status != 0)
			osDelay(IMU_TASK_INIT_RETRY_MS);
	} while (init_status != 0);

	wake_tick = osKernelGetTickCount();

	for (;;) {
		(void)icm45686_app_process((float)APP_ICM45686_PERIOD_MS / 1000.0f, NULL);
		wake_tick += APP_ICM45686_PERIOD_MS;
		osDelayUntil(wake_tick);
	}
}
