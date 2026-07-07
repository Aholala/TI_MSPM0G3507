/**
 * @file bsp_buzzer.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief
 * @version 1.0
 * @date 2026-07-07
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "board_config.h"
#include "module_buzzer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_BUZZER_DEFAULT_ACTIVE_LOW BOARD_BUZZER_DEFAULT_ACTIVE_LOW

void BspBuzzer_Bind(Buzzer_Ops *ops);
void BspBuzzer_GetDefaultConfig(Buzzer_Config *cfg);
void BspBuzzer_WriteLevel(uint8_t level);
uint8_t BspBuzzer_ReadLevel(void);

#ifdef __cplusplus
}
#endif

#endif
