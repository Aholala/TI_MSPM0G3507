/**
 * @file module_key.c
 * @brief Non-blocking key debounce module.
 */

#include "module_key.h"

#include <stddef.h>

#define KEY_DEFAULT_DEBOUNCE_MS 20u
#define KEY_DEFAULT_LONG_PRESS_MS 1000u

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t since_ms)
{
    return now_ms - since_ms;
}

static Key_State level_to_state(uint8_t level, uint8_t active_low)
{
    if (active_low != 0u)
    {
        return (level == 0u) ? KEY_STATE_PRESSED : KEY_STATE_RELEASED;
    }

    return (level != 0u) ? KEY_STATE_PRESSED : KEY_STATE_RELEASED;
}

static uint8_t read_level(const Key *key)
{
    if ((key == NULL) || (key->ops.read_level == NULL))
    {
        return 0u;
    }

    return (key->ops.read_level(key->ops.user_ctx) != 0u) ? 1u : 0u;
}

void Key_Init(Key *key, const Key_Ops *ops, const Key_Config *cfg, uint32_t now_ms)
{
    uint8_t level;

    if (key == NULL)
    {
        return;
    }

    key->ops.read_level = NULL;
    key->ops.user_ctx = NULL;
    if (ops != NULL)
    {
        key->ops = *ops;
    }

    key->cfg.active_low = 1u;
    key->cfg.debounce_ms = KEY_DEFAULT_DEBOUNCE_MS;
    key->cfg.long_press_ms = KEY_DEFAULT_LONG_PRESS_MS;
    if (cfg != NULL)
    {
        key->cfg = *cfg;
    }

    level = read_level(key);
    key->last_raw_level = level;
    key->stable_level = level;
    key->last_change_time_ms = now_ms;
    key->pressed_time_ms = now_ms;
    key->long_press_reported = 0u;
    key->state = level_to_state(level, key->cfg.active_low);
    key->events = KEY_EVENT_NONE;
}

uint8_t Key_Update(Key *key, uint32_t now_ms)
{
    uint8_t raw_level;
    Key_State new_state;

    if (key == NULL)
    {
        return KEY_EVENT_NONE;
    }

    key->events = KEY_EVENT_NONE;
    raw_level = read_level(key);

    if (raw_level != key->last_raw_level)
    {
        key->last_raw_level = raw_level;
        key->last_change_time_ms = now_ms;
    }

    if ((raw_level != key->stable_level) &&
        (elapsed_ms(now_ms, key->last_change_time_ms) >= key->cfg.debounce_ms))
    {
        key->stable_level = raw_level;
        new_state = level_to_state(key->stable_level, key->cfg.active_low);

        if (new_state != key->state)
        {
            key->state = new_state;

            if (new_state == KEY_STATE_PRESSED)
            {
                key->pressed_time_ms = now_ms;
                key->long_press_reported = 0u;
                key->events |= KEY_EVENT_PRESSED;
            }
            else
            {
                key->long_press_reported = 0u;
                key->events |= KEY_EVENT_RELEASED;
            }
        }
    }

    if ((key->state == KEY_STATE_PRESSED) &&
        (key->long_press_reported == 0u) &&
        (key->cfg.long_press_ms > 0u) &&
        (elapsed_ms(now_ms, key->pressed_time_ms) >= key->cfg.long_press_ms))
    {
        key->long_press_reported = 1u;
        key->events |= KEY_EVENT_LONG_PRESSED;
    }

    return key->events;
}

uint8_t Key_GetEvents(const Key *key)
{
    return (key != NULL) ? key->events : KEY_EVENT_NONE;
}

uint8_t Key_PopEvents(Key *key)
{
    uint8_t events;

    if (key == NULL)
    {
        return KEY_EVENT_NONE;
    }

    events = key->events;
    key->events = KEY_EVENT_NONE;
    return events;
}

Key_State Key_GetState(const Key *key)
{
    return (key != NULL) ? key->state : KEY_STATE_RELEASED;
}

uint8_t Key_IsPressed(const Key *key)
{
    return (Key_GetState(key) == KEY_STATE_PRESSED) ? 1u : 0u;
}
