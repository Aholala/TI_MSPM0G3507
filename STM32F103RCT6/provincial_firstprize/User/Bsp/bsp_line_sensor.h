/**
 * @file bsp_line_sensor.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef BSP_LINE_SENSOR_H
#define BSP_LINE_SENSOR_H

#include <stdint.h>

#include "main.h"
#include "module_tracker.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_LINE_SENSOR_DEFAULT_ACTIVE_LOW 1u

uint8_t BspLineSensor_ReadRawMask(void *user_ctx);
void BspLineSensor_Bind(LineTracker_Ops *ops);

#ifdef __cplusplus
}
#endif

#endif
