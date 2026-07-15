#include "bsp_key.h"

#include <stddef.h>

#include "board_config.h"
#include "ti_msp_dl_config.h"

typedef struct
{
    GPIO_Regs *port;
    uint32_t pin;
} BspKey_Hw;

static const BspKey_Hw key_hw[BSP_KEY_COUNT] = {
    {KEY_PORT, KEY_KEY1_PIN},
    {KEY_PORT, KEY_KEY2_PIN},
    {KEY_PORT, KEY_KEY3_PIN},
};

static uint8_t read_level(void *user_ctx)
{
    const BspKey_Hw *hw = (const BspKey_Hw *)user_ctx;

    return ((hw != NULL) && (DL_GPIO_readPins(hw->port, hw->pin) != 0u)) ? 1u : 0u;
}

void BspKey_Bind(BspKey_Id key_id, Key_Ops *ops)
{
    if ((ops != NULL) && (key_id < BSP_KEY_COUNT))
    {
        ops->read_level = read_level;
        ops->user_ctx = (void *)&key_hw[key_id];
    }
}

void BspKey_GetDefaultConfig(Key_Config *cfg)
{
    if (cfg != NULL)
    {
        cfg->active_low = BOARD_KEY_DEFAULT_ACTIVE_LOW;
        cfg->debounce_ms = BOARD_KEY_DEFAULT_DEBOUNCE_MS;
        cfg->long_press_ms = BOARD_KEY_DEFAULT_LONG_PRESS_MS;
    }
}

uint8_t BspKey_ReadLevel(BspKey_Id key_id)
{
    return (key_id < BSP_KEY_COUNT) ? read_level((void *)&key_hw[key_id]) : 0u;
}
