#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int16_t g_control_task_base_speed;
extern int16_t g_control_task_turn;

void StartcontrolTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
