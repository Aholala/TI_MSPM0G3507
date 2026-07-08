/**
 * @file bsp_key.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bsp_key.h"

#include <stddef.h>

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} BspKey_Hw;

static const BspKey_Hw key_hw[BSP_KEY_COUNT] = {
    {BOARD_KEY_PA8_GPIO_PORT, BOARD_KEY_PA8_GPIO_PIN},
    {BOARD_KEY_PC13_GPIO_PORT, BOARD_KEY_PC13_GPIO_PIN},
    {BOARD_KEY_PC14_GPIO_PORT, BOARD_KEY_PC14_GPIO_PIN},
};

static uint8_t read_level(void *user_ctx)
{
    const BspKey_Hw *hw = (const BspKey_Hw *)user_ctx;

    if (hw == NULL)
    {
        return 0u;
    }

    return (HAL_GPIO_ReadPin(hw->port, hw->pin) == GPIO_PIN_SET) ? 1u : 0u;
}

void BspKey_Bind(BspKey_Id key_id, Key_Ops *ops)
{
    if ((ops == NULL) || (key_id >= BSP_KEY_COUNT))
    {
        return;
    }

    ops->read_level = read_level;
    ops->user_ctx = (void *)&key_hw[key_id];
}

void BspKey_GetDefaultConfig(Key_Config *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    cfg->active_low = BSP_KEY_DEFAULT_ACTIVE_LOW;
    cfg->debounce_ms = BSP_KEY_DEFAULT_DEBOUNCE_MS;
    cfg->long_press_ms = BSP_KEY_DEFAULT_LONG_PRESS_MS;
}

uint8_t BspKey_ReadLevel(BspKey_Id key_id)
{
    if (key_id >= BSP_KEY_COUNT)
    {
        return 0u;
    }

    return read_level((void *)&key_hw[key_id]);
}
