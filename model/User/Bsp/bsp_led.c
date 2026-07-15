#include "bsp_led.h"

#include <stddef.h>

/* No status LED pin was assigned on this board; retain logical state only. */
static uint8_t led_level;

static void write_level(void *user_ctx, uint8_t level)
{
    (void)user_ctx;
    led_level = (level != 0u) ? 1u : 0u;
}

static uint8_t read_level(void *user_ctx)
{
    (void)user_ctx;
    return led_level;
}

void BspLed_Bind(Led_Ops *ops)
{
    if (ops != NULL)
    {
        ops->write_level = write_level;
        ops->read_level = read_level;
        ops->user_ctx = NULL;
    }
}

void BspLed_GetDefaultConfig(Led_Config *cfg)
{
    if (cfg != NULL)
    {
        cfg->active_low = 0u;
    }
}
