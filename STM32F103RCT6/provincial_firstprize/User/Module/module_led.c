/**
 * @file module_led.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "module_led.h"

#include <stddef.h>

static uint8_t enabled_to_level(uint8_t enabled, uint8_t active_low)
{
    uint8_t active_level = (active_low != 0u) ? 0u : 1u;

    return (enabled != 0u) ? active_level : (uint8_t)(active_level ^ 1u);
}

void Led_Init(Led *led, const Led_Ops *ops, const Led_Config *cfg)
{
    if (led == NULL)
    {
        return;
    }

    led->ops.write_level = NULL;
    led->ops.read_level = NULL;
    led->ops.user_ctx = NULL;
    if (ops != NULL)
    {
        led->ops = *ops;
    }

    led->cfg.active_low = 0u;
    if (cfg != NULL)
    {
        led->cfg = *cfg;
    }

    led->enabled = 0u;
    Led_Off(led);
}

void Led_Set(Led *led, uint8_t enabled)
{
    if (led == NULL)
    {
        return;
    }

    led->enabled = (enabled != 0u) ? 1u : 0u;
    if (led->ops.write_level != NULL)
    {
        led->ops.write_level(led->ops.user_ctx,
                             enabled_to_level(led->enabled, led->cfg.active_low));
    }
}

void Led_On(Led *led)
{
    Led_Set(led, 1u);
}

void Led_Off(Led *led)
{
    Led_Set(led, 0u);
}

void Led_Toggle(Led *led)
{
    if (led == NULL)
    {
        return;
    }

    Led_Set(led, (led->enabled == 0u) ? 1u : 0u);
}

uint8_t Led_IsOn(const Led *led)
{
    return (led != NULL) ? led->enabled : 0u;
}
