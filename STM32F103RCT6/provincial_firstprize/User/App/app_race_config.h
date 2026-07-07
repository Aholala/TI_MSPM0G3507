/**
 * @file app_race_config.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef APP_RACE_CONFIG_H
#define APP_RACE_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_RACE_CONFIG_MIN_MODE 1u
#define APP_RACE_CONFIG_MAX_MODE 4u
#define APP_RACE_CONFIG_MIN_LAPS 1u
#define APP_RACE_CONFIG_MAX_LAPS 4u
#define APP_RACE_CONFIG_PERIOD_MS 20u

typedef enum
{
    APP_RACE_STATE_IDLE = 0,
    APP_RACE_STATE_READY,
    APP_RACE_STATE_RUNNING,
    APP_RACE_STATE_FINISHED
} AppRace_State;

typedef struct
{
    uint8_t mode;
    uint8_t target_laps;
    uint8_t current_lap;
    AppRace_State state;
} AppRaceConfig_Snapshot;

extern AppRaceConfig_Snapshot g_debug_race_config_snapshot;

void AppRaceConfig_Init(uint32_t now_ms);
void AppRaceConfig_RunOnce(uint32_t now_ms);
AppRaceConfig_Snapshot AppRaceConfig_GetSnapshot(void);
uint8_t AppRaceConfig_GetMode(void);
uint8_t AppRaceConfig_GetTargetLaps(void);
uint8_t AppRaceConfig_IsRunning(void);
void AppRaceConfig_SetFinished(void);
void AppRaceConfig_NotifyPoint(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
