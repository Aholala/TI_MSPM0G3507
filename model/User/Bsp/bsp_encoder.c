/**
 * @file bsp_encoder.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "bsp_encoder.h"

#include "tim.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    int16_t last_count;
    int32_t total_count;
    uint8_t inverted;
} BspEncoder;

static BspEncoder encoders[BSP_ENCODER_COUNT] = {
    {
        &htim4,
        0,
        0,
        BOARD_ENCODER_LEFT_INVERTED,
    },
    {
        &htim8,
        0,
        0,
        BOARD_ENCODER_RIGHT_INVERTED,
    },
};

static BspEncoder *get_encoder(BspEncoder_Id encoder_id)
{
    if (encoder_id >= BSP_ENCODER_COUNT)
    {
        return 0;
    }

    return &encoders[encoder_id];
}

void BspEncoder_Init(void)
{
    uint8_t index;

    for (index = 0u; index < BSP_ENCODER_COUNT; ++index)
    {
        __HAL_TIM_SET_COUNTER(encoders[index].htim, 0u);
        encoders[index].last_count = 0;
        encoders[index].total_count = 0;
        (void)HAL_TIM_Encoder_Start(encoders[index].htim, TIM_CHANNEL_ALL);
    }
}

void BspEncoder_Reset(BspEncoder_Id encoder_id)
{
    BspEncoder *encoder = get_encoder(encoder_id);

    if (encoder == 0)
    {
        return;
    }

    __HAL_TIM_SET_COUNTER(encoder->htim, 0u);
    encoder->last_count = 0;
    encoder->total_count = 0;
}

int16_t BspEncoder_ReadDelta(BspEncoder_Id encoder_id)
{
    BspEncoder *encoder = get_encoder(encoder_id);
    int16_t current_count;
    int16_t delta;

    if (encoder == 0)
    {
        return 0;
    }

    current_count = (int16_t)__HAL_TIM_GET_COUNTER(encoder->htim);
    delta = (int16_t)(current_count - encoder->last_count);
    encoder->last_count = current_count;

    if (encoder->inverted != 0u)
    {
        delta = (int16_t)-delta;
    }

    encoder->total_count += delta;

    return delta;
}

int32_t BspEncoder_GetTotal(BspEncoder_Id encoder_id)
{
    BspEncoder *encoder = get_encoder(encoder_id);

    return (encoder != 0) ? encoder->total_count : 0;
}

uint16_t BspEncoder_GetPulsesPerRevolution(void)
{
    return BOARD_ENCODER_OUTPUT_PULSES_PER_REV;
}

int32_t BspEncoder_PulsesToMilliRevolutions(int32_t pulses)
{
    int32_t scaled;

    if (BOARD_ENCODER_OUTPUT_PULSES_PER_REV == 0u)
    {
        return 0;
    }

    scaled = pulses * 1000;
    if (scaled >= 0)
    {
        scaled += (int32_t)(BOARD_ENCODER_OUTPUT_PULSES_PER_REV / 2u);
    }
    else
    {
        scaled -= (int32_t)(BOARD_ENCODER_OUTPUT_PULSES_PER_REV / 2u);
    }

    return scaled / (int32_t)BOARD_ENCODER_OUTPUT_PULSES_PER_REV;
}

int32_t BspEncoder_PulsesToMillimeters(int32_t pulses)
{
    int64_t distance_um;
    int64_t divisor;

    if (BOARD_ENCODER_OUTPUT_PULSES_PER_REV == 0u)
    {
        return 0;
    }

    distance_um = (int64_t)pulses * (int64_t)BOARD_WHEEL_CIRCUMFERENCE_UM;
    divisor = (int64_t)BOARD_ENCODER_OUTPUT_PULSES_PER_REV * 1000;

    if (distance_um >= 0)
    {
        distance_um += divisor / 2;
    }
    else
    {
        distance_um -= divisor / 2;
    }

    return (int32_t)(distance_um / divisor);
}

int32_t BspEncoder_GetTotalMilliRevolutions(BspEncoder_Id encoder_id)
{
    return BspEncoder_PulsesToMilliRevolutions(BspEncoder_GetTotal(encoder_id));
}

int32_t BspEncoder_GetTotalMillimeters(BspEncoder_Id encoder_id)
{
    return BspEncoder_PulsesToMillimeters(BspEncoder_GetTotal(encoder_id));
}
