/**
 * @file bsp_key.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef BSP_KEY_H
#define BSP_KEY_H

#include "board_config.h"
#include "module_key.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_KEY_DEFAULT_ACTIVE_LOW BOARD_KEY_DEFAULT_ACTIVE_LOW
#define BSP_KEY_DEFAULT_DEBOUNCE_MS BOARD_KEY_DEFAULT_DEBOUNCE_MS
#define BSP_KEY_DEFAULT_LONG_PRESS_MS BOARD_KEY_DEFAULT_LONG_PRESS_MS

typedef enum
{
    BSP_KEY_PA8 = 0,
    BSP_KEY_PC13,
    BSP_KEY_PC14,
    BSP_KEY_COUNT
} BspKey_Id;

void BspKey_Bind(BspKey_Id key_id, Key_Ops *ops);
void BspKey_GetDefaultConfig(Key_Config *cfg);
uint8_t BspKey_ReadLevel(BspKey_Id key_id);

#ifdef __cplusplus
}
#endif

#endif
