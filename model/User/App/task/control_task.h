/**
 * @file control_task.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int16_t g_control_task_base_speed;
extern int16_t g_control_task_turn;
extern volatile int32_t g_line_follow_kp_x1000;
extern volatile int32_t g_line_follow_kd_x1000;
/* Set to -1 in the debugger if the vehicle steers away from the line. */
extern volatile int8_t g_line_follow_steering_sign;
extern volatile int32_t g_line_exit_target_degrees;
extern volatile int32_t g_line_exit_track_width_mm;
extern volatile int32_t g_line_exit_angle_mdeg;
extern volatile int16_t g_line_exit_turn;
extern volatile int32_t g_q1_target_distance_mm;
extern volatile int32_t g_q1_cruise_rpm;
extern volatile int32_t g_q1_travel_mm;
extern volatile int32_t g_q1_heading_error_mm;
extern volatile uint8_t g_q1_heading_pid_enable;
extern volatile int32_t g_q1_heading_pid_kp_x1000;
extern volatile int32_t g_q1_heading_pid_ki_x1000;
extern volatile int32_t g_q1_heading_pid_kd_x1000;
extern volatile int8_t g_q1_heading_steering_sign;
extern volatile int16_t g_q1_heading_max_turn_delta;
extern volatile float g_q1_heading_target_deg;
extern volatile float g_q1_heading_current_deg;
extern volatile float g_q1_heading_error_deg;
extern volatile float g_q1_heading_integral_deg_s;
extern volatile float g_q1_heading_output_delta;
extern volatile int32_t g_q2_straight_distance_mm;
extern volatile int32_t g_q2_arc_min_distance_mm;
extern volatile uint8_t g_q2_arc_lost_confirm_cycles;
extern volatile uint8_t g_q2_phase;
extern volatile int32_t g_q2_segment_travel_mm;
extern volatile int32_t g_q2_arc_travel_mm;

void StartcontrolTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif
