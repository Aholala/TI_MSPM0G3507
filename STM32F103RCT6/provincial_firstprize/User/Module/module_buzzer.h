/**
 * @file module_buzzer.h
 * @brief Portable buzzer control module.
 */

#ifndef MODULE_BUZZER_H
#define MODULE_BUZZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    void (*write_level)(void *user_ctx, uint8_t level);
    uint8_t (*read_level)(void *user_ctx);
    void *user_ctx;
} Buzzer_Ops;

typedef struct
{
    uint8_t active_low;
} Buzzer_Config;

typedef struct
{
    Buzzer_Ops ops;
    Buzzer_Config cfg;
    uint8_t enabled;
} Buzzer;

void Buzzer_Init(Buzzer *buzzer, const Buzzer_Ops *ops, const Buzzer_Config *cfg);
void Buzzer_Set(Buzzer *buzzer, uint8_t enabled);
void Buzzer_On(Buzzer *buzzer);
void Buzzer_Off(Buzzer *buzzer);
void Buzzer_Toggle(Buzzer *buzzer);
uint8_t Buzzer_IsOn(const Buzzer *buzzer);

#ifdef __cplusplus
}
#endif

#endif
