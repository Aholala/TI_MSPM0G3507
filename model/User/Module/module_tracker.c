/**
 * @file line_tracker.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "module_tracker.h"

#include <stddef.h>

#define LINE_TRACKER_ALL_MASK 0xFFu

static const int16_t sensor_weights[LINE_TRACKER_SENSOR_COUNT] = {
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
};

static uint8_t get_active_mask(uint8_t raw_mask, uint8_t active_low)
{
    if (active_low != 0u)
    {
        return (uint8_t)(~raw_mask) & LINE_TRACKER_ALL_MASK;
    }

    return raw_mask & LINE_TRACKER_ALL_MASK;
}

void LineTracker_Init(LineTracker *tracker,
                      const LineTracker_Ops *ops,
                      const LineTracker_Config *cfg)
{
    if (tracker == NULL)
    {
        return;
    }

    tracker->ops.read_raw_mask = NULL;
    tracker->ops.user_ctx = NULL;
    if (ops != NULL)
    {
        tracker->ops = *ops;
    }

    tracker->cfg.active_low = 0u;
    tracker->cfg.lost_position = 0;
    if (cfg != NULL)
    {
        tracker->cfg = *cfg;
    }

    tracker->raw_mask = 0u;
    tracker->active_mask = 0u;
    tracker->position = tracker->cfg.lost_position;
    tracker->error = tracker->position;
    tracker->status = LINE_TRACKER_LOST;
}

LineTracker_Status LineTracker_Update(LineTracker *tracker)
{
    int32_t weighted_sum = 0;
    int32_t active_count = 0;
    uint8_t index;

    if ((tracker == NULL) || (tracker->ops.read_raw_mask == NULL))
    {
        return LINE_TRACKER_LOST;
    }

    tracker->raw_mask = tracker->ops.read_raw_mask(tracker->ops.user_ctx);
    tracker->active_mask = get_active_mask(tracker->raw_mask, tracker->cfg.active_low);

    if (tracker->active_mask == 0u)
    {
        tracker->position = tracker->cfg.lost_position;
        tracker->error = tracker->position;
        tracker->status = LINE_TRACKER_LOST;
        return tracker->status;
    }

    if (tracker->active_mask == LINE_TRACKER_ALL_MASK)
    {
        tracker->position = 0;
        tracker->error = 0;
        tracker->status = LINE_TRACKER_FULL;
        return tracker->status;
    }

    for (index = 0u; index < LINE_TRACKER_SENSOR_COUNT; ++index)
    {
        if ((tracker->active_mask & (uint8_t)(1u << index)) != 0u)
        {
            weighted_sum += sensor_weights[index];
            ++active_count;
        }
    }

    tracker->position = (int16_t)(weighted_sum / active_count);
    tracker->error = tracker->position;
    tracker->status = LINE_TRACKER_OK;

    return tracker->status;
}

uint8_t LineTracker_GetRawMask(const LineTracker *tracker)
{
    return (tracker != NULL) ? tracker->raw_mask : 0u;
}

uint8_t LineTracker_GetActiveMask(const LineTracker *tracker)
{
    return (tracker != NULL) ? tracker->active_mask : 0u;
}

int16_t LineTracker_GetPosition(const LineTracker *tracker)
{
    return (tracker != NULL) ? tracker->position : 0;
}

int16_t LineTracker_GetError(const LineTracker *tracker)
{
    return (tracker != NULL) ? tracker->error : 0;
}

LineTracker_Status LineTracker_GetStatus(const LineTracker *tracker)
{
    return (tracker != NULL) ? tracker->status : LINE_TRACKER_LOST;
}
