/**
 * @file module_led.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef MODULE_LED_H
#define MODULE_LED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    void (*write_level)(void *user_ctx, uint8_t level);
    uint8_t (*read_level)(void *user_ctx);
    void *user_ctx;
} Led_Ops;

typedef struct
{
    uint8_t active_low;
} Led_Config;

typedef struct
{
    Led_Ops ops;
    Led_Config cfg;
    uint8_t enabled;
} Led;

void Led_Init(Led *led, const Led_Ops *ops, const Led_Config *cfg);
void Led_Set(Led *led, uint8_t enabled);
void Led_On(Led *led);
void Led_Off(Led *led);
void Led_Toggle(Led *led);
uint8_t Led_IsOn(const Led *led);

#ifdef __cplusplus
}
#endif

#endif
