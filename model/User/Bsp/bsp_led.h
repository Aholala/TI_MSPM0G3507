#ifndef BSP_LED_H
#define BSP_LED_H

#include "module_led.h"

void BspLed_Bind(Led_Ops *ops);
void BspLed_GetDefaultConfig(Led_Config *cfg);

#endif
