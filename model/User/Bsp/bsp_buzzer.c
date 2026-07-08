/**
 * @file bsp_buzzer.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bsp_buzzer.h"

#include <stddef.h>

static void write_level(void *user_ctx, uint8_t level)
{
    (void)user_ctx;

    BspBuzzer_WriteLevel(level);
}

static uint8_t read_level(void *user_ctx)
{
    (void)user_ctx;

    return BspBuzzer_ReadLevel();
}

void BspBuzzer_Bind(Buzzer_Ops *ops)
{
    if (ops == NULL)
    {
        return;
    }

    ops->write_level = write_level;
    ops->read_level = read_level;
    ops->user_ctx = NULL;
}

void BspBuzzer_GetDefaultConfig(Buzzer_Config *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    cfg->active_low = BSP_BUZZER_DEFAULT_ACTIVE_LOW;
}

void BspBuzzer_WriteLevel(uint8_t level)
{
    HAL_GPIO_WritePin(BOARD_BUZZER_GPIO_PORT,
                      BOARD_BUZZER_GPIO_PIN,
                      (level != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t BspBuzzer_ReadLevel(void)
{
    return (HAL_GPIO_ReadPin(BOARD_BUZZER_GPIO_PORT, BOARD_BUZZER_GPIO_PIN) == GPIO_PIN_SET) ? 1u : 0u;
}
