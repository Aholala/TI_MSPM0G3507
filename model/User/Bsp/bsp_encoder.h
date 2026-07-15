#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BSP_ENCODER_LEFT = 0,
    BSP_ENCODER_RIGHT,
    BSP_ENCODER_COUNT
} BspEncoder_Id;

/*
 * E1 is treated as the left encoder and E2 as the right encoder.
 * Both A and B phases are decoded on both edges in the GPIOB interrupt.
 */
void BspEncoder_Init(void);
void BspEncoder_Reset(BspEncoder_Id encoder_id);
int16_t BspEncoder_ReadDelta(BspEncoder_Id encoder_id);
int32_t BspEncoder_GetTotal(BspEncoder_Id encoder_id);
uint16_t BspEncoder_GetPulsesPerRevolution(void);
int32_t BspEncoder_PulsesToMilliRevolutions(int32_t pulses);
int32_t BspEncoder_PulsesToMillimeters(int32_t pulses);
int32_t BspEncoder_GetTotalMilliRevolutions(BspEncoder_Id encoder_id);
int32_t BspEncoder_GetTotalMillimeters(BspEncoder_Id encoder_id);

#ifdef __cplusplus
}
#endif

#endif
