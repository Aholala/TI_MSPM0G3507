#include "bsp_line_sensor.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

typedef struct
{
    GPIO_Regs *port;
    uint32_t pin;
} BspLineSensor_Pin;

static const BspLineSensor_Pin line_sensor_pins[LINE_TRACKER_SENSOR_COUNT] = {
    {LINE_SENSOR_SENSOR1_PORT, LINE_SENSOR_SENSOR1_PIN},
    {LINE_SENSOR_SENSOR2_PORT, LINE_SENSOR_SENSOR2_PIN},
    {LINE_SENSOR_SENSOR3_PORT, LINE_SENSOR_SENSOR3_PIN},
    {LINE_SENSOR_SENSOR4_PORT, LINE_SENSOR_SENSOR4_PIN},
    {LINE_SENSOR_SENSOR5_PORT, LINE_SENSOR_SENSOR5_PIN},
    {LINE_SENSOR_SENSOR6_PORT, LINE_SENSOR_SENSOR6_PIN},
    {LINE_SENSOR_SENSOR7_PORT, LINE_SENSOR_SENSOR7_PIN},
};

volatile uint8_t g_debug_line_sensor_raw_mask;
volatile uint8_t g_debug_line_sensor_filtered_mask;
static uint8_t g_raw_history_1;
static uint8_t g_raw_history_2;
static uint8_t g_history_initialized;

uint8_t BspLineSensor_ReadRawMask(void *user_ctx)
{
    uint8_t index;
    uint8_t mask = 0u;

    (void)user_ctx;
    for (index = 0u; index < LINE_TRACKER_SENSOR_COUNT; ++index)
    {
        if (DL_GPIO_readPins(line_sensor_pins[index].port,
                            line_sensor_pins[index].pin) != 0u)
        {
            mask |= (uint8_t)(1u << index);
        }
    }
    g_debug_line_sensor_raw_mask = mask;

    if (g_history_initialized == 0u)
    {
        g_raw_history_1 = mask;
        g_raw_history_2 = mask;
        g_history_initialized = 1u;
    }

    /* Per-sensor majority vote over three 5 ms samples. This rejects one
     * sample of comparator chatter at a black/white boundary. */
    g_debug_line_sensor_filtered_mask =
        (uint8_t)((mask & g_raw_history_1) |
                  (mask & g_raw_history_2) |
                  (g_raw_history_1 & g_raw_history_2));
    g_raw_history_2 = g_raw_history_1;
    g_raw_history_1 = mask;

    return g_debug_line_sensor_filtered_mask;
}

void BspLineSensor_Bind(LineTracker_Ops *ops)
{
    if (ops != NULL)
    {
        ops->read_raw_mask = BspLineSensor_ReadRawMask;
        ops->user_ctx = NULL;
    }
}
