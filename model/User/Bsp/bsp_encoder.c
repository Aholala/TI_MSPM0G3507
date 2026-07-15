#include "bsp_encoder.h"

#include <limits.h>
#include <stddef.h>

#include "board_config.h"
#include "ti_msp_dl_config.h"

typedef struct
{
    uint32_t pin_a;
    uint32_t pin_b;
    volatile int32_t total_count;
    int32_t last_reported_count;
    volatile uint8_t previous_state;
    uint8_t inverted;
} BspEncoder;

/* Index is (previous AB << 2) | current AB. Invalid jumps count as zero. */
static const int8_t quadrature_delta[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0,
};

static BspEncoder encoders[BSP_ENCODER_COUNT] = {
    {
        ENCODER_E1A_PIN,
        ENCODER_E1B_PIN,
        0,
        0,
        0,
        BOARD_ENCODER_LEFT_INVERTED,
    },
    {
        ENCODER_E2A_PIN,
        ENCODER_E2B_PIN,
        0,
        0,
        0,
        BOARD_ENCODER_RIGHT_INVERTED,
    },
};

static BspEncoder *get_encoder(BspEncoder_Id encoder_id)
{
    if (encoder_id >= BSP_ENCODER_COUNT)
    {
        return NULL;
    }

    return &encoders[encoder_id];
}

static uint8_t read_state(const BspEncoder *encoder)
{
    uint32_t pins = DL_GPIO_readPins(ENCODER_PORT,
                                     encoder->pin_a | encoder->pin_b);
    uint8_t state = 0u;

    if ((pins & encoder->pin_a) != 0u)
    {
        state |= 2u;
    }
    if ((pins & encoder->pin_b) != 0u)
    {
        state |= 1u;
    }

    return state;
}

static void update_encoder(BspEncoder *encoder)
{
    uint8_t current_state = read_state(encoder);
    uint8_t transition = (uint8_t)((encoder->previous_state << 2u) |
                                   current_state);
    int8_t delta = quadrature_delta[transition];

    encoder->previous_state = current_state;
    if (encoder->inverted != 0u)
    {
        delta = (int8_t)-delta;
    }
    encoder->total_count += delta;
}

static uint32_t enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void leave_critical(uint32_t primask)
{
    if (primask == 0u)
    {
        __enable_irq();
    }
}

void BspEncoder_Init(void)
{
    uint8_t index;

    for (index = 0u; index < BSP_ENCODER_COUNT; ++index)
    {
        encoders[index].total_count = 0;
        encoders[index].last_reported_count = 0;
        encoders[index].previous_state = read_state(&encoders[index]);
    }

    DL_GPIO_clearInterruptStatus(ENCODER_PORT,
                                 ENCODER_E1A_PIN | ENCODER_E1B_PIN |
                                 ENCODER_E2A_PIN | ENCODER_E2B_PIN);
    NVIC_ClearPendingIRQ(ENCODER_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
}

void BspEncoder_Reset(BspEncoder_Id encoder_id)
{
    BspEncoder *encoder = get_encoder(encoder_id);
    uint32_t primask;

    if (encoder == NULL)
    {
        return;
    }

    primask = enter_critical();
    encoder->total_count = 0;
    encoder->last_reported_count = 0;
    encoder->previous_state = read_state(encoder);
    leave_critical(primask);
}

int16_t BspEncoder_ReadDelta(BspEncoder_Id encoder_id)
{
    BspEncoder *encoder = get_encoder(encoder_id);
    int32_t delta;
    uint32_t primask;

    if (encoder == NULL)
    {
        return 0;
    }

    primask = enter_critical();
    delta = encoder->total_count - encoder->last_reported_count;
    encoder->last_reported_count = encoder->total_count;
    leave_critical(primask);

    if (delta > INT16_MAX)
    {
        return INT16_MAX;
    }
    if (delta < INT16_MIN)
    {
        return INT16_MIN;
    }
    return (int16_t)delta;
}

int32_t BspEncoder_GetTotal(BspEncoder_Id encoder_id)
{
    BspEncoder *encoder = get_encoder(encoder_id);
    int32_t total;
    uint32_t primask;

    if (encoder == NULL)
    {
        return 0;
    }

    primask = enter_critical();
    total = encoder->total_count;
    leave_critical(primask);
    return total;
}

uint16_t BspEncoder_GetPulsesPerRevolution(void)
{
    return BOARD_ENCODER_OUTPUT_PULSES_PER_REV;
}

int32_t BspEncoder_PulsesToMilliRevolutions(int32_t pulses)
{
    int64_t scaled;

    if (BOARD_ENCODER_OUTPUT_PULSES_PER_REV == 0u)
    {
        return 0;
    }

    scaled = (int64_t)pulses * 1000;
    if (scaled >= 0)
    {
        scaled += BOARD_ENCODER_OUTPUT_PULSES_PER_REV / 2u;
    }
    else
    {
        scaled -= BOARD_ENCODER_OUTPUT_PULSES_PER_REV / 2u;
    }
    return (int32_t)(scaled / BOARD_ENCODER_OUTPUT_PULSES_PER_REV);
}

int32_t BspEncoder_PulsesToMillimeters(int32_t pulses)
{
    int64_t distance_um;
    int64_t divisor;

    if (BOARD_ENCODER_OUTPUT_PULSES_PER_REV == 0u)
    {
        return 0;
    }

    distance_um = (int64_t)pulses * BOARD_WHEEL_CIRCUMFERENCE_UM;
    divisor = (int64_t)BOARD_ENCODER_OUTPUT_PULSES_PER_REV * 1000;
    distance_um += (distance_um >= 0) ? (divisor / 2) : -(divisor / 2);
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

void GROUP1_IRQHandler(void)
{
    const uint32_t encoder_pins = ENCODER_E1A_PIN | ENCODER_E1B_PIN |
                                  ENCODER_E2A_PIN | ENCODER_E2B_PIN;
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(ENCODER_PORT,
                                                          encoder_pins);

    DL_GPIO_clearInterruptStatus(ENCODER_PORT, pending);

    if ((pending & (ENCODER_E1A_PIN | ENCODER_E1B_PIN)) != 0u)
    {
        update_encoder(&encoders[BSP_ENCODER_LEFT]);
    }
    if ((pending & (ENCODER_E2A_PIN | ENCODER_E2B_PIN)) != 0u)
    {
        update_encoder(&encoders[BSP_ENCODER_RIGHT]);
    }
}
