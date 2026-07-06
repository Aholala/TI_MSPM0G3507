/**
 * @file bsp_line_sensor.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bsp_line_sensor.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} BspLineSensor_Pin;

static const BspLineSensor_Pin line_sensor_pins[LINE_TRACKER_SENSOR_COUNT] = {
    {GPIOC, GPIO_PIN_0},
    {GPIOC, GPIO_PIN_1},
    {GPIOC, GPIO_PIN_2},
    {GPIOC, GPIO_PIN_3},
    {GPIOC, GPIO_PIN_5},
    {GPIOC, GPIO_PIN_8},
    {GPIOC, GPIO_PIN_9},
    {GPIOC, GPIO_PIN_12},
};

uint8_t BspLineSensor_ReadRawMask(void *user_ctx)
{
    uint8_t index;
    uint8_t mask = 0u;

    (void)user_ctx;

    for (index = 0u; index < LINE_TRACKER_SENSOR_COUNT; ++index)
    {
        if (HAL_GPIO_ReadPin(line_sensor_pins[index].port,
                             line_sensor_pins[index].pin) == GPIO_PIN_SET)
        {
            mask |= (uint8_t)(1u << index);
        }
    }

    return mask;
}

void BspLineSensor_Bind(LineTracker_Ops *ops)
{
    if (ops == NULL)
    {
        return;
    }

    ops->read_raw_mask = BspLineSensor_ReadRawMask;
    ops->user_ctx = NULL;
}
