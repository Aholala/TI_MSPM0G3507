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
#include "ti_msp_dl_config.h"

#define APP_RACE_NOTIFY_KEY_MS 20u
#define APP_RACE_NOTIFY_POINT_MS 150u
#define APP_RACE_NOTIFY_FINISH_MS 300u
#define APP_RACE_NOTIFY_POINT_LOCKOUT_MS 800u
#define APP_RACE_OLED_REFRESH_MS 50u

typedef enum
{
    APP_RACE_KEY_LAP = BSP_KEY_1,   /* PA16 */
    APP_RACE_KEY_START = BSP_KEY_2, /* PA8  */
    APP_RACE_KEY_MODE = BSP_KEY_3,  /* PA18 */
    APP_RACE_KEY_COUNT = BSP_KEY_COUNT
} AppRace_KeyId;

static Key g_keys[APP_RACE_KEY_COUNT];
static volatile uint8_t g_key_press_count[APP_RACE_KEY_COUNT];
static volatile uint8_t g_key_timer_ready;
static Buzzer g_buzzer;
static Led g_led;
static AppRaceConfig_Snapshot g_snapshot;
AppRaceConfig_Snapshot g_debug_race_config_snapshot;
volatile uint32_t g_debug_key_timer_count;
static volatile uint32_t g_notify_off_time_ms;
static volatile uint32_t g_point_notify_next_allowed_ms;
static uint32_t g_last_oled_time_ms;
static volatile uint8_t g_display_dirty;

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t since_ms)
{
    return now_ms - since_ms;
}

static void notify_on(uint32_t now_ms, uint16_t duration_ms)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    /* Publish the new deadline before enabling the outputs.  Together with
     * the short critical section this prevents notify_update(), running in a
     * different task, from switching off a newly started notification using
     * the previous expired deadline. */
    g_notify_off_time_ms = now_ms + duration_ms;
    Buzzer_On(&g_buzzer);
    Led_On(&g_led);
    if (primask == 0u)
    {
        __enable_irq();
    }
}

static void notify_update(uint32_t now_ms)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if ((Buzzer_IsOn(&g_buzzer) != 0u) &&
        ((int32_t)(now_ms - g_notify_off_time_ms) >= 0))
    {
        Buzzer_Off(&g_buzzer);
        Led_Off(&g_led);
    }
    if (primask == 0u)
    {
        __enable_irq();
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
    g_snapshot.target_laps = (mode == 4u) ? 4u : APP_RACE_CONFIG_MIN_LAPS;
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
    if (g_snapshot.mode == 4u)
    {
        g_snapshot.target_laps = 4u;
        g_snapshot.current_lap = 0u;
        g_snapshot.state = APP_RACE_STATE_READY;
        g_display_dirty = 1u;
        return;
    }

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
    switch (g_snapshot.state)
    {
    case APP_RACE_STATE_READY:
        g_snapshot.current_lap = 0u;
        g_snapshot.state = APP_RACE_STATE_RUNNING;
        break;

    case APP_RACE_STATE_RUNNING:
        g_snapshot.state = APP_RACE_STATE_STOPPED;
        break;

    case APP_RACE_STATE_STOPPED:
        g_snapshot.state = APP_RACE_STATE_RUNNING;
        break;

    case APP_RACE_STATE_FINISHED:
    case APP_RACE_STATE_IDLE:
    default:
        g_snapshot.current_lap = 0u;
        g_snapshot.state = APP_RACE_STATE_READY;
        break;
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

static void oled_prepare_frame(const AppRaceConfig_Snapshot *snapshot)
{
    g_debug_race_config_snapshot = *snapshot;

    OLED_Clear();
    OLED_ShowString(0, 0, "Q:", OLED_8X16);
    OLED_ShowNum(16, 0, snapshot->mode, 1u, OLED_8X16);
    OLED_ShowString(40, 0, "Lap:", OLED_8X16);
    OLED_ShowNum(72, 0, snapshot->target_laps, 1u, OLED_8X16);

    OLED_ShowString(0, 16, "Now:", OLED_8X16);
    OLED_ShowNum(32, 16, snapshot->current_lap, 1u, OLED_8X16);
    OLED_ShowString(64, 16, (char *)state_text(snapshot->state), OLED_8X16);

    OLED_ShowString(0, 32, "Route:", OLED_6X8);
    OLED_ShowString(36, 32, (char *)route_text(snapshot->mode), OLED_6X8);
    OLED_ShowString(0, 48, "K1 Lap K2 R/S K3 Q", OLED_6X8);
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

    g_snapshot.mode = 1u;
    g_snapshot.target_laps = APP_RACE_CONFIG_MIN_LAPS;
    g_snapshot.current_lap = 0u;
    g_snapshot.state = APP_RACE_STATE_READY;
    g_notify_off_time_ms = now_ms;
    g_point_notify_next_allowed_ms = now_ms;
    g_last_oled_time_ms = now_ms;
    g_display_dirty = 1u;
    g_key_timer_ready = 1u;
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

}

void AppRaceConfig_DisplayRunOnce(uint32_t now_ms)
{
    static uint8_t initialized;
    static uint8_t transfer_active;
    static uint8_t page;
    AppRaceConfig_Snapshot snapshot;
    uint32_t primask;

    if (initialized == 0u)
    {
        /* Initialization is intentionally done in the low-priority display
         * task so its settling delay cannot hold up control or key tasks. */
        OLED_Init();
        initialized = 1u;
        g_display_dirty = 1u;
    }

    if (transfer_active == 0u)
    {
        if ((g_display_dirty == 0u) &&
            (elapsed_ms(now_ms, g_last_oled_time_ms) < APP_RACE_OLED_REFRESH_MS))
        {
            return;
        }

        primask = __get_PRIMASK();
        __disable_irq();
        snapshot = g_snapshot;
        g_display_dirty = 0u;
        if (primask == 0u)
        {
            __enable_irq();
        }

        oled_prepare_frame(&snapshot);
        g_last_oled_time_ms = now_ms;
        page = 0u;
        transfer_active = 1u;
    }

    /* Send only one 8-pixel page per call. Higher-priority control, encoder
     * and key tasks can run between pages instead of waiting for a full frame. */
    OLED_UpdateArea(0, (int16_t)(page * 8u), 128u, 8u);
    ++page;
    if (page >= 8u)
    {
        transfer_active = 0u;
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

void AppRaceConfig_SetFinished(uint32_t now_ms)
{
    /* Completion is a one-shot event.  Repeated endpoint samples must not
     * restart the finish tone. */
    if (g_snapshot.state == APP_RACE_STATE_FINISHED)
    {
        return;
    }
    g_snapshot.state = APP_RACE_STATE_FINISHED;
    g_display_dirty = 1u;
    notify_on(now_ms, APP_RACE_NOTIFY_FINISH_MS);
}

void AppRaceConfig_NotifyPoint(uint32_t now_ms)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t accepted = 0u;

    __disable_irq();
    if ((int32_t)(now_ms - g_point_notify_next_allowed_ms) >= 0)
    {
        g_point_notify_next_allowed_ms =
            now_ms + APP_RACE_NOTIFY_POINT_LOCKOUT_MS;
        accepted = 1u;
    }
    if (primask == 0u)
    {
        __enable_irq();
    }

    if (accepted == 0u)
    {
        return;
    }
    notify_on(now_ms, APP_RACE_NOTIFY_POINT_MS);
}

void AppRaceConfig_SetLapProgress(uint8_t current_lap, uint8_t target_laps)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_snapshot.current_lap = current_lap;
    g_snapshot.target_laps = target_laps;
    g_display_dirty = 1u;
    if (primask == 0u)
    {
        __enable_irq();
    }
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
