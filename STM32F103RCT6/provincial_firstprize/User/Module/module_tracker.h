/**
 * @file line_tracker.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LINE_TRACKER_SENSOR_COUNT 7u

typedef enum
{
    LINE_TRACKER_OK = 0,
    LINE_TRACKER_LOST,
    LINE_TRACKER_FULL
} LineTracker_Status;

typedef uint8_t (*LineTracker_ReadRawMaskFn)(void *user_ctx);

typedef struct
{
    LineTracker_ReadRawMaskFn read_raw_mask;
    void *user_ctx;
} LineTracker_Ops;

typedef struct
{
    uint8_t active_low;
    int16_t lost_position;
} LineTracker_Config;

typedef struct
{
    LineTracker_Ops ops;
    LineTracker_Config cfg;
    uint8_t raw_mask;
    uint8_t active_mask;
    int16_t position;
    int16_t error;
    LineTracker_Status status;
} LineTracker;

void LineTracker_Init(LineTracker *tracker,
                      const LineTracker_Ops *ops,
                      const LineTracker_Config *cfg);

LineTracker_Status LineTracker_Update(LineTracker *tracker);

uint8_t LineTracker_GetRawMask(const LineTracker *tracker);
uint8_t LineTracker_GetActiveMask(const LineTracker *tracker);
int16_t LineTracker_GetPosition(const LineTracker *tracker);
int16_t LineTracker_GetError(const LineTracker *tracker);
LineTracker_Status LineTracker_GetStatus(const LineTracker *tracker);

#ifdef __cplusplus
}
#endif

#endif
