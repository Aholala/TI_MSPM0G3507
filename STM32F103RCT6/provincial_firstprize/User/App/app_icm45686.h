#ifndef __ICM45686_APP_H
#define __ICM45686_APP_H

#include <stdint.h>

#include "../lib/lib_attitude.h"
#include "../module/module_icm45686.h"

#define APP_ICM45686_PERIOD_MS 5U

typedef struct {
	imu_solution_t solution;
	uint8_t initialized;
	uint8_t data_ready;
	int last_init_status;
	int last_process_status;
	uint32_t update_count;
} app_icm45686_snapshot_t;

int icm45686_app_init(const icm45686_config_t *config);
int icm45686_app_init_ex(const icm45686_config_t *imu_config,
                         const imu_attitude_config_t *attitude_config);
void icm45686_app_int1_irq_handler(void);
uint8_t icm45686_app_is_data_ready(void);
int icm45686_app_read_data(icm45686_data_t *data);
int icm45686_app_read_data_if_ready(icm45686_data_t *data);
int icm45686_app_process(float dt_s, imu_solution_t *solution);
const imu_solution_t *icm45686_app_get_solution(void);
icm45686_module_t *icm45686_app_get_module(void);
app_icm45686_snapshot_t icm45686_app_get_snapshot(void);
float icm45686_app_get_yaw_deg(void);

#endif
