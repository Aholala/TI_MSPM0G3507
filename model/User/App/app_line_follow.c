/**
 * @file app_line_follow.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "app_line_follow.h"

#include "bsp_line_sensor.h"
#include "main.h"

static LineTracker g_line_tracker;
static AppLineFollow_Snapshot g_line_follow_snapshot;
AppLineFollow_Snapshot g_debug_line_follow_snapshot;

static uint32_t enter_snapshot_lock(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    return primask;
}

static void leave_snapshot_lock(uint32_t primask)
{
    if (primask == 0u)
    {
        __enable_irq();
    }
}

static void update_snapshot(void)
{
    uint32_t primask = enter_snapshot_lock();

    g_line_follow_snapshot.raw_mask = LineTracker_GetRawMask(&g_line_tracker);
    g_line_follow_snapshot.active_mask = LineTracker_GetActiveMask(&g_line_tracker);
    g_line_follow_snapshot.position = LineTracker_GetPosition(&g_line_tracker);
    g_line_follow_snapshot.error = LineTracker_GetError(&g_line_tracker);
    g_line_follow_snapshot.status = LineTracker_GetStatus(&g_line_tracker);
    g_debug_line_follow_snapshot = g_line_follow_snapshot;

    leave_snapshot_lock(primask);
}

void AppLineFollow_Init(void)
{
    LineTracker_Ops line_sensor_ops;
    LineTracker_Config line_tracker_cfg = {
        .active_low = BSP_LINE_SENSOR_DEFAULT_ACTIVE_LOW,
        .lost_position = 0,
    };

    BspLineSensor_Bind(&line_sensor_ops);
    LineTracker_Init(&g_line_tracker, &line_sensor_ops, &line_tracker_cfg);
    update_snapshot();
}

LineTracker_Status AppLineFollow_RunOnce(void)
{
    LineTracker_Status status = LineTracker_Update(&g_line_tracker);

    update_snapshot();

    return status;
}

AppLineFollow_Snapshot AppLineFollow_GetSnapshot(void)
{
    AppLineFollow_Snapshot snapshot;
    uint32_t primask = enter_snapshot_lock();

    snapshot = g_line_follow_snapshot;
    leave_snapshot_lock(primask);

    return snapshot;
}

int16_t AppLineFollow_GetError(void)
{
    int16_t error;
    uint32_t primask = enter_snapshot_lock();

    error = g_line_follow_snapshot.error;
    leave_snapshot_lock(primask);

    return error;
}

LineTracker_Status AppLineFollow_GetStatus(void)
{
    LineTracker_Status status;
    uint32_t primask = enter_snapshot_lock();

    status = g_line_follow_snapshot.status;
    leave_snapshot_lock(primask);

    return status;
}
