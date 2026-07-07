#include "icm45686_app.h"

static icm45686_module_t s_icm45686;

int icm45686_app_init(const icm45686_config_t *config)
{
	return icm45686_module_init(&s_icm45686, &icm45686_bsp_default, config);
}

void icm45686_app_int1_irq_handler(void)
{
	icm45686_module_int1_irq_handler(&s_icm45686);
}

uint8_t icm45686_app_is_data_ready(void)
{
	return icm45686_module_is_data_ready(&s_icm45686);
}

int icm45686_app_read_data(icm45686_data_t *data)
{
	return icm45686_module_read_data(&s_icm45686, data);
}

int icm45686_app_read_data_if_ready(icm45686_data_t *data)
{
	return icm45686_module_read_data_if_ready(&s_icm45686, data);
}

icm45686_module_t *icm45686_app_get_module(void)
{
	return &s_icm45686;
}
