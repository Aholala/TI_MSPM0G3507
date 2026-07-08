/**
 * @file bsp_encoder.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BSP_ENCODER_LEFT = 0,
    BSP_ENCODER_RIGHT,
    BSP_ENCODER_COUNT
} BspEncoder_Id;

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
