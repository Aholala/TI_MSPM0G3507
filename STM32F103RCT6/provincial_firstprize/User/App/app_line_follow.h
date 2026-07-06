/**
 * @file app_line_follow.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef APP_LINE_FOLLOW_H
#define APP_LINE_FOLLOW_H

#include <stdint.h>

#include "module_tracker.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_LINE_FOLLOW_PERIOD_MS 5u

typedef struct
{
    uint8_t raw_mask;
    uint8_t active_mask;
    int16_t position;
    int16_t error;
    LineTracker_Status status;
} AppLineFollow_Snapshot;

void AppLineFollow_Init(void);
LineTracker_Status AppLineFollow_RunOnce(void);

AppLineFollow_Snapshot AppLineFollow_GetSnapshot(void);
int16_t AppLineFollow_GetError(void);
LineTracker_Status AppLineFollow_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif
