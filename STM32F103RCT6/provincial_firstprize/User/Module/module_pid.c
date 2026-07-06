#include "module_pid.h"

#include <stddef.h>

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

void PidController_Init(PidController *pid, const PidController_Config *cfg)
{
    if (pid == NULL)
    {
        return;
    }

    pid->cfg.kp = 0;
    pid->cfg.ki = 0;
    pid->cfg.kd = 0;
    pid->cfg.scale = 1;
    pid->cfg.integral_min = 0;
    pid->cfg.integral_max = 0;
    pid->cfg.output_min = 0;
    pid->cfg.output_max = 0;
    if (cfg != NULL)
    {
        pid->cfg = *cfg;
        if (pid->cfg.scale == 0)
        {
            pid->cfg.scale = 1;
        }
    }

    PidController_Reset(pid);
}

void PidController_SetConfig(PidController *pid, const PidController_Config *cfg)
{
    if ((pid == NULL) || (cfg == NULL))
    {
        return;
    }

    pid->cfg = *cfg;
    if (pid->cfg.scale == 0)
    {
        pid->cfg.scale = 1;
    }
}

void PidController_Reset(PidController *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->target = 0;
    pid->feedback = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
}

int32_t PidController_Run(PidController *pid, int32_t target, int32_t feedback)
{
    int32_t raw_output;

    if (pid == NULL)
    {
        return 0;
    }

    pid->target = target;
    pid->feedback = feedback;
    pid->error = target - feedback;
    pid->derivative = pid->error - pid->last_error;
    pid->last_error = pid->error;

    pid->integral += pid->error;
    if (pid->cfg.integral_min < pid->cfg.integral_max)
    {
        pid->integral = clamp_i32(pid->integral,
                                  pid->cfg.integral_min,
                                  pid->cfg.integral_max);
    }

    raw_output = (pid->cfg.kp * pid->error) +
                 (pid->cfg.ki * pid->integral) +
                 (pid->cfg.kd * pid->derivative);
    pid->output = raw_output / pid->cfg.scale;

    if (pid->cfg.output_min < pid->cfg.output_max)
    {
        pid->output = clamp_i32(pid->output,
                                pid->cfg.output_min,
                                pid->cfg.output_max);
    }

    return pid->output;
}
