/**
 * @file app_race_config.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "app_race_config.h"

#include "bsp_buzzer.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_oled.h"
#include "module_buzzer.h"
#include "module_key.h"
#include "module_led.h"

#define APP_RACE_NOTIFY_KEY_MS 60u
#define APP_RACE_NOTIFY_POINT_MS 150u
#define APP_RACE_NOTIFY_FINISH_MS 300u
#define APP_RACE_OLED_REFRESH_MS 100u

typedef enum
{
    APP_RACE_KEY_MODE = 0,
    APP_RACE_KEY_LAP,
    APP_RACE_KEY_START,
    APP_RACE_KEY_COUNT
} AppRace_KeyId;

static Key g_keys[APP_RACE_KEY_COUNT];
static volatile uint8_t g_key_press_count[APP_RACE_KEY_COUNT];
static volatile uint8_t g_key_timer_ready;
static Buzzer g_buzzer;
static Led g_led;
static AppRaceConfig_Snapshot g_snapshot;
AppRaceConfig_Snapshot g_debug_race_config_snapshot;
volatile uint32_t g_debug_key_timer_count;
static uint32_t g_notify_off_time_ms;
static uint32_t g_last_oled_time_ms;
static uint8_t g_display_dirty;

static uint8_t default_laps_for_mode(uint8_t mode)
{
    return (mode == 4u) ? 4u : 1u;
}

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t since_ms)
{
    return now_ms - since_ms;
}

static void notify_on(uint32_t now_ms, uint16_t duration_ms)
{
    Buzzer_On(&g_buzzer);
    Led_On(&g_led);
    g_notify_off_time_ms = now_ms + duration_ms;
}

static void notify_update(uint32_t now_ms)
{
    if ((Buzzer_IsOn(&g_buzzer) != 0u) &&
        ((int32_t)(now_ms - g_notify_off_time_ms) >= 0))
    {
        Buzzer_Off(&g_buzzer);
        Led_Off(&g_led);
    }
}

static void set_mode(uint8_t mode)
{
    if (mode > APP_RACE_CONFIG_MAX_MODE)
    {
        mode = APP_RACE_CONFIG_MIN_MODE;
    }

    if (mode < APP_RACE_CONFIG_MIN_MODE)
    {
        mode = APP_RACE_CONFIG_MIN_MODE;
    }

    g_snapshot.mode = mode;
    g_snapshot.target_laps = default_laps_for_mode(mode);
    g_snapshot.current_lap = 0u;
    g_snapshot.state = APP_RACE_STATE_READY;
    g_display_dirty = 1u;
}

static void next_mode(void)
{
    set_mode((uint8_t)(g_snapshot.mode + 1u));
}

static void next_lap(void)
{
    ++g_snapshot.target_laps;
    if (g_snapshot.target_laps > APP_RACE_CONFIG_MAX_LAPS)
    {
        g_snapshot.target_laps = APP_RACE_CONFIG_MIN_LAPS;
    }

    g_snapshot.current_lap = 0u;
    g_snapshot.state = APP_RACE_STATE_READY;
    g_display_dirty = 1u;
}

static void toggle_start_stop(void)
{
    if (g_snapshot.state == APP_RACE_STATE_RUNNING)
    {
        g_snapshot.state = APP_RACE_STATE_STOPPED;
    }
    else
    {
        if ((g_snapshot.state == APP_RACE_STATE_READY) ||
            (g_snapshot.state == APP_RACE_STATE_FINISHED))
        {
            g_snapshot.current_lap = 0u;
        }

        g_snapshot.state = APP_RACE_STATE_RUNNING;
    }

    g_display_dirty = 1u;
}

static const char *state_text(AppRace_State state)
{
    switch (state)
    {
    case APP_RACE_STATE_READY:
        return "READY";
    case APP_RACE_STATE_RUNNING:
        return "RUN";
    case APP_RACE_STATE_STOPPED:
        return "STOP";
    case APP_RACE_STATE_FINISHED:
        return "DONE";
    case APP_RACE_STATE_IDLE:
    default:
        return "IDLE";
    }
}

static const char *route_text(uint8_t mode)
{
    switch (mode)
    {
    case 1u:
        return "A-B";
    case 2u:
        return "A-B-C-D-A";
    case 3u:
    case 4u:
        return "A-C-B-D-A";
    default:
        return "--";
    }
}

static void oled_draw(void)
{
    g_debug_race_config_snapshot = g_snapshot;

    OLED_Clear();
    OLED_ShowString(0, 0, "Q:", OLED_8X16);
    OLED_ShowNum(16, 0, g_snapshot.mode, 1u, OLED_8X16);
    OLED_ShowString(40, 0, "Lap:", OLED_8X16);
    OLED_ShowNum(72, 0, g_snapshot.target_laps, 1u, OLED_8X16);

    OLED_ShowString(0, 16, "Now:", OLED_8X16);
    OLED_ShowNum(32, 16, g_snapshot.current_lap, 1u, OLED_8X16);
    OLED_ShowString(64, 16, (char *)state_text(g_snapshot.state), OLED_8X16);

    OLED_ShowString(0, 32, "Route:", OLED_6X8);
    OLED_ShowString(36, 32, (char *)route_text(g_snapshot.mode), OLED_6X8);
    OLED_ShowString(0, 48, "K1 Mode K2 Lap K3 R/S", OLED_6X8);
    OLED_Update();
    g_display_dirty = 0u;
}

static uint8_t pop_key_pressed_event(AppRace_KeyId key_id)
{
    uint8_t event = KEY_EVENT_NONE;
    uint32_t primask;

    if (key_id >= APP_RACE_KEY_COUNT)
    {
        return KEY_EVENT_NONE;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (g_key_press_count[key_id] > 0u)
    {
        --g_key_press_count[key_id];
        event = KEY_EVENT_PRESSED;
    }
    if (primask == 0u)
    {
        __enable_irq();
    }

    return event;
}

void AppRaceConfig_Init(uint32_t now_ms)
{
    Key_Ops key_ops;
    Key_Config key_cfg;
    Buzzer_Ops buzzer_ops;
    Buzzer_Config buzzer_cfg;
    Led_Ops led_ops;
    Led_Config led_cfg;
    uint8_t index;

    BspKey_GetDefaultConfig(&key_cfg);

    for (index = 0u; index < APP_RACE_KEY_COUNT; ++index)
    {
        BspKey_Bind((BspKey_Id)index, &key_ops);
        Key_Init(&g_keys[index], &key_ops, &key_cfg, now_ms);
        g_key_press_count[index] = 0u;
    }

    BspBuzzer_Bind(&buzzer_ops);
    BspBuzzer_GetDefaultConfig(&buzzer_cfg);
    Buzzer_Init(&g_buzzer, &buzzer_ops, &buzzer_cfg);

    BspLed_Bind(&led_ops);
    BspLed_GetDefaultConfig(&led_cfg);
    Led_Init(&g_led, &led_ops, &led_cfg);

    OLED_Init();

    g_snapshot.mode = 1u;
    g_snapshot.target_laps = default_laps_for_mode(g_snapshot.mode);
    g_snapshot.current_lap = 0u;
    g_snapshot.state = APP_RACE_STATE_READY;
    g_notify_off_time_ms = now_ms;
    g_last_oled_time_ms = now_ms;
    g_display_dirty = 1u;
    g_key_timer_ready = 1u;
    oled_draw();
}

void AppRaceConfig_RunOnce(uint32_t now_ms)
{
    uint8_t events;

    events = pop_key_pressed_event(APP_RACE_KEY_MODE);
    if ((events & KEY_EVENT_PRESSED) != 0u)
    {
        next_mode();
        notify_on(now_ms, APP_RACE_NOTIFY_KEY_MS);
    }

    events = pop_key_pressed_event(APP_RACE_KEY_LAP);
    if ((events & KEY_EVENT_PRESSED) != 0u)
    {
        next_lap();
        notify_on(now_ms, APP_RACE_NOTIFY_KEY_MS);
    }

    events = pop_key_pressed_event(APP_RACE_KEY_START);
    if ((events & KEY_EVENT_PRESSED) != 0u)
    {
        toggle_start_stop();
        notify_on(now_ms, APP_RACE_NOTIFY_KEY_MS);
    }

    notify_update(now_ms);

    if ((g_display_dirty != 0u) ||
        (elapsed_ms(now_ms, g_last_oled_time_ms) >= APP_RACE_OLED_REFRESH_MS))
    {
        g_last_oled_time_ms = now_ms;
        oled_draw();
    }
}

AppRaceConfig_Snapshot AppRaceConfig_GetSnapshot(void)
{
    return g_snapshot;
}

uint8_t AppRaceConfig_GetMode(void)
{
    return g_snapshot.mode;
}

uint8_t AppRaceConfig_GetTargetLaps(void)
{
    return g_snapshot.target_laps;
}

uint8_t AppRaceConfig_IsRunning(void)
{
    return (g_snapshot.state == APP_RACE_STATE_RUNNING) ? 1u : 0u;
}

void AppRaceConfig_SetFinished(void)
{
    g_snapshot.state = APP_RACE_STATE_FINISHED;
    g_display_dirty = 1u;
    notify_on(g_last_oled_time_ms, APP_RACE_NOTIFY_FINISH_MS);
}

void AppRaceConfig_NotifyPoint(uint32_t now_ms)
{
    notify_on(now_ms, APP_RACE_NOTIFY_POINT_MS);
}

void AppRaceConfig_KeyTimer1ms(uint32_t now_ms)
{
    uint8_t index;
    uint8_t events;

    if (g_key_timer_ready == 0u)
    {
        return;
    }

    ++g_debug_key_timer_count;
    for (index = 0u; index < APP_RACE_KEY_COUNT; ++index)
    {
        events = Key_Update(&g_keys[index], now_ms);
        if ((events & KEY_EVENT_PRESSED) != 0u)
        {
            if (g_key_press_count[index] < 255u)
            {
                ++g_key_press_count[index];
            }
        }
    }
}
