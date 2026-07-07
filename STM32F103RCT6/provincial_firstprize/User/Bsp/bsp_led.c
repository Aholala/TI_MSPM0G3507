/**
 * @file bsp_led.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bsp_led.h"

#include <stddef.h>

static void write_level(void *user_ctx, uint8_t level)
{
    (void)user_ctx;

    BspLed_WriteLevel(level);
}

static uint8_t read_level(void *user_ctx)
{
    (void)user_ctx;

    return BspLed_ReadLevel();
}

void BspLed_Bind(Led_Ops *ops)
{
    if (ops == NULL)
    {
        return;
    }

    ops->write_level = write_level;
    ops->read_level = read_level;
    ops->user_ctx = NULL;
}

void BspLed_GetDefaultConfig(Led_Config *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    cfg->active_low = BSP_LED_DEFAULT_ACTIVE_LOW;
}

void BspLed_WriteLevel(uint8_t level)
{
    HAL_GPIO_WritePin(BOARD_LED_GPIO_PORT,
                      BOARD_LED_GPIO_PIN,
                      (level != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t BspLed_ReadLevel(void)
{
    return (HAL_GPIO_ReadPin(BOARD_LED_GPIO_PORT, BOARD_LED_GPIO_PIN) == GPIO_PIN_SET) ? 1u : 0u;
}
