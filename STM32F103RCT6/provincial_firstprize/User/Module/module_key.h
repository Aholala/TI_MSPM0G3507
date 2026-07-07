/**
 * @file module_key.h
 * @brief Non-blocking key debounce module.
 */

#ifndef MODULE_KEY_H
#define MODULE_KEY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    KEY_STATE_RELEASED = 0,
    KEY_STATE_PRESSED
} Key_State;

typedef enum
{
    KEY_EVENT_NONE = 0x00u,
    KEY_EVENT_PRESSED = 0x01u,
    KEY_EVENT_RELEASED = 0x02u,
    KEY_EVENT_LONG_PRESSED = 0x04u
} Key_Event;

typedef uint8_t (*Key_ReadLevelFn)(void *user_ctx);

typedef struct
{
    Key_ReadLevelFn read_level;
    void *user_ctx;
} Key_Ops;

typedef struct
{
    uint8_t active_low;
    uint16_t debounce_ms;
    uint16_t long_press_ms;
} Key_Config;

typedef struct
{
    Key_Ops ops;
    Key_Config cfg;
    uint8_t last_raw_level;
    uint8_t stable_level;
    uint32_t last_change_time_ms;
    uint32_t pressed_time_ms;
    uint8_t long_press_reported;
    Key_State state;
    uint8_t events;
} Key;

void Key_Init(Key *key, const Key_Ops *ops, const Key_Config *cfg, uint32_t now_ms);
uint8_t Key_Update(Key *key, uint32_t now_ms);
uint8_t Key_GetEvents(const Key *key);
uint8_t Key_PopEvents(Key *key);
Key_State Key_GetState(const Key *key);
uint8_t Key_IsPressed(const Key *key);

#ifdef __cplusplus
}
#endif

#endif
