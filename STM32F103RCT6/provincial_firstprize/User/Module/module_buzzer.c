/**
 * @file module_buzzer.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "module_buzzer.h"

#include <stddef.h>

static uint8_t enabled_to_level(uint8_t enabled, uint8_t active_low)
{
    uint8_t active_level = (active_low != 0u) ? 0u : 1u;

    return (enabled != 0u) ? active_level : (uint8_t)(active_level ^ 1u);
}

void Buzzer_Init(Buzzer *buzzer, const Buzzer_Ops *ops, const Buzzer_Config *cfg)
{
    if (buzzer == NULL)
    {
        return;
    }

    buzzer->ops.write_level = NULL;
    buzzer->ops.read_level = NULL;
    buzzer->ops.user_ctx = NULL;
    if (ops != NULL)
    {
        buzzer->ops = *ops;
    }

    buzzer->cfg.active_low = 0u;
    if (cfg != NULL)
    {
        buzzer->cfg = *cfg;
    }

    buzzer->enabled = 0u;
    Buzzer_Off(buzzer);
}

void Buzzer_Set(Buzzer *buzzer, uint8_t enabled)
{
    if (buzzer == NULL)
    {
        return;
    }

    buzzer->enabled = (enabled != 0u) ? 1u : 0u;
    if (buzzer->ops.write_level != NULL)
    {
        buzzer->ops.write_level(buzzer->ops.user_ctx,
                                enabled_to_level(buzzer->enabled, buzzer->cfg.active_low));
    }
}

void Buzzer_On(Buzzer *buzzer)
{
    Buzzer_Set(buzzer, 1u);
}

void Buzzer_Off(Buzzer *buzzer)
{
    Buzzer_Set(buzzer, 0u);
}

void Buzzer_Toggle(Buzzer *buzzer)
{
    if (buzzer == NULL)
    {
        return;
    }

    Buzzer_Set(buzzer, (buzzer->enabled == 0u) ? 1u : 0u);
}

uint8_t Buzzer_IsOn(const Buzzer *buzzer)
{
    return (buzzer != NULL) ? buzzer->enabled : 0u;
}
