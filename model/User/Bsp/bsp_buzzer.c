#include "bsp_buzzer.h"

#include <stddef.h>

#include "board_config.h"
#include "ti_msp_dl_config.h"

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
    if (ops != NULL)
    {
        ops->write_level = write_level;
        ops->read_level = read_level;
        ops->user_ctx = NULL;
    }
}

void BspBuzzer_GetDefaultConfig(Buzzer_Config *cfg)
{
    if (cfg != NULL)
    {
        cfg->active_low = BOARD_BUZZER_DEFAULT_ACTIVE_LOW;
    }
}

void BspBuzzer_WriteLevel(uint8_t level)
{
    if (level != 0u)
    {
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_BUZZER_OUT_PIN);
    }
    else
    {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BUZZER_OUT_PIN);
    }
}

uint8_t BspBuzzer_ReadLevel(void)
{
    return (DL_GPIO_readPins(BUZZER_PORT, BUZZER_BUZZER_OUT_PIN) != 0u) ? 1u : 0u;
}
