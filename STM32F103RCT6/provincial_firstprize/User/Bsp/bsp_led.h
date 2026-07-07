/**
 * @file bsp_led.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef BSP_LED_H
#define BSP_LED_H

#include "board_config.h"
#include "module_led.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_LED_DEFAULT_ACTIVE_LOW BOARD_LED_DEFAULT_ACTIVE_LOW

void BspLed_Bind(Led_Ops *ops);
void BspLed_GetDefaultConfig(Led_Config *cfg);
void BspLed_WriteLevel(uint8_t level);
uint8_t BspLed_ReadLevel(void);

#ifdef __cplusplus
}
#endif

#endif
