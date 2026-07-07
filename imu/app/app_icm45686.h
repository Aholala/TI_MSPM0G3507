#ifndef __ICM45686_APP_H
#define __ICM45686_APP_H

#include <stdint.h>

#include "../module/icm45686_module.h"

int icm45686_app_init(const icm45686_config_t *config);
void icm45686_app_int1_irq_handler(void);
uint8_t icm45686_app_is_data_ready(void);
int icm45686_app_read_data(icm45686_data_t *data);
int icm45686_app_read_data_if_ready(icm45686_data_t *data);
icm45686_module_t *icm45686_app_get_module(void);

#endif
