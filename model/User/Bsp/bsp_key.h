#ifndef BSP_KEY_H
#define BSP_KEY_H

#include "board_config.h"
#include "module_key.h"

#define BSP_KEY_DEFAULT_ACTIVE_LOW BOARD_KEY_DEFAULT_ACTIVE_LOW
#define BSP_KEY_DEFAULT_DEBOUNCE_MS BOARD_KEY_DEFAULT_DEBOUNCE_MS
#define BSP_KEY_DEFAULT_LONG_PRESS_MS BOARD_KEY_DEFAULT_LONG_PRESS_MS

typedef enum
{
    BSP_KEY_1 = 0,
    BSP_KEY_2,
    BSP_KEY_3,
    BSP_KEY_COUNT
} BspKey_Id;

void BspKey_Bind(BspKey_Id key_id, Key_Ops *ops);
void BspKey_GetDefaultConfig(Key_Config *cfg);
uint8_t BspKey_ReadLevel(BspKey_Id key_id);

#endif
