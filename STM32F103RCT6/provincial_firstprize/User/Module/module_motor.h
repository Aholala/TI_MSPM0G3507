#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DC_MOTOR_MAX_SPEED 1000

typedef enum
{
    DC_MOTOR_COAST = 0,
    DC_MOTOR_FORWARD,
    DC_MOTOR_REVERSE,
    DC_MOTOR_BRAKE
} DcMotor_Mode;

typedef struct
{
    void (*set_mode)(void *user_ctx, DcMotor_Mode mode);
    void (*set_duty)(void *user_ctx, uint16_t duty);
    void *user_ctx;
} DcMotor_Ops;

typedef struct
{
    uint8_t inverted;
} DcMotor_Config;

typedef struct
{
    DcMotor_Ops ops;
    DcMotor_Config cfg;
    int16_t speed;
} DcMotor;

void DcMotor_Init(DcMotor *motor, const DcMotor_Ops *ops, const DcMotor_Config *cfg);
void DcMotor_SetSpeed(DcMotor *motor, int16_t speed);
void DcMotor_Coast(DcMotor *motor);
void DcMotor_Brake(DcMotor *motor);
int16_t DcMotor_GetSpeed(const DcMotor *motor);

#ifdef __cplusplus
}
#endif

#endif
