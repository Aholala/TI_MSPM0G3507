#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t scale;
    int32_t integral_min;
    int32_t integral_max;
    int32_t output_min;
    int32_t output_max;
} PidController_Config;

typedef struct
{
    PidController_Config cfg;
    int32_t target;
    int32_t feedback;
    int32_t error;
    int32_t last_error;
    int32_t integral;
    int32_t derivative;
    int32_t output;
} PidController;

void PidController_Init(PidController *pid, const PidController_Config *cfg);
void PidController_SetConfig(PidController *pid, const PidController_Config *cfg);
void PidController_Reset(PidController *pid);
int32_t PidController_Run(PidController *pid, int32_t target, int32_t feedback);

#ifdef __cplusplus
}
#endif

#endif
