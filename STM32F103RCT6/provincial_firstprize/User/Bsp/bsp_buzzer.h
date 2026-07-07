/**
 * @file bsp_buzzer.h
 * @brief Board buzzer GPIO adapter.
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
