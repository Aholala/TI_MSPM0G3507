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
        0u,
    },
    {
        &htim8,
        0,
        0,
        0u,
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
