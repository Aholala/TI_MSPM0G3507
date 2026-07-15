#ifndef BSP_LINE_SENSOR_H
#define BSP_LINE_SENSOR_H

#include "board_config.h"
#include "module_tracker.h"

#define BSP_LINE_SENSOR_DEFAULT_ACTIVE_LOW BOARD_LINE_SENSOR_DEFAULT_ACTIVE_LOW

extern volatile uint8_t g_debug_line_sensor_raw_mask;
extern volatile uint8_t g_debug_line_sensor_filtered_mask;

uint8_t BspLineSensor_ReadRawMask(void *user_ctx);
void BspLineSensor_Bind(LineTracker_Ops *ops);

#endif
