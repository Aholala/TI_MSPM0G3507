/**
 * @file control_task.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief
 * @version 1.0
 * @date 2026-07-06
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "control_task.h"

#include "app_chassis.h"
#include "app_line_follow.h"
#include "app_race_config.h"
#include "board_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lib_rate_filter.h"

#define CONTROL_TASK_PERIOD_MS BOARD_CONTROL_TASK_PERIOD_MS
#define CONTROL_TASK_START_DELAY_MS 500u
#define CONTROL_TASK_LINE_GAIN_SCALE 1000
#define CONTROL_TASK_DEFAULT_BASE_SPEED ((int16_t)BOARD_CHASSIS_DEFAULT_SPEED_DELTA)
#define CONTROL_TASK_LINE_FILTER_DIV 3u
#define CONTROL_TASK_LINE_ERROR_DEADBAND 600
#define CONTROL_TASK_TURN_SLEW_STEP 4
#define CONTROL_TASK_SPEED_REDUCTION_DIV 125
#define CONTROL_TASK_MIN_SPEED_DIV 2
#define CONTROL_TASK_MAX_TURN_NUM 2
#define CONTROL_TASK_MAX_TURN_DIV 3
#define CONTROL_TASK_EXIT_ALIGN_SPEED_NUM 2
#define CONTROL_TASK_EXIT_ALIGN_SPEED_DIV 3
#define CONTROL_Q2_C_ALIGN_SETTLE_CYCLES 10u
#define CONTROL_TASK_ENABLE_CORNER_RECOVERY 0u
#define CONTROL_TASK_LEFT_BRANCH_MASK 0x03u
#define CONTROL_TASK_RIGHT_BRANCH_MASK 0x60u
#define CONTROL_TASK_CENTER_REACQUIRE_MASK 0x1Cu
#define CONTROL_POINT_CENTER_MASK 0x1Cu
#define CONTROL_POINT_MAX_ERROR 1500
#define CONTROL_POINT_CONFIRM_CYCLES 3u
#define CONTROL_TASK_CORNER_HINT_HOLD_CYCLES 20u
#define CONTROL_TASK_CORNER_ENTRY_DISTANCE_MM 35
#define CONTROL_TASK_CORNER_ENTRY_SPEED_DIV 2
#define CONTROL_TASK_CORNER_TURN_SPEED_DIV 2
#define CONTROL_TASK_CORNER_MIN_TURN_CYCLES 8u
#define CONTROL_TASK_CORNER_REACQUIRE_CYCLES 3u
#define CONTROL_TASK_CORNER_TIMEOUT_CYCLES 150u

typedef enum
{
    CONTROL_CORNER_NONE = 0,
    CONTROL_CORNER_LEFT = -1,
    CONTROL_CORNER_RIGHT = 1
} ControlCornerDirection;

typedef enum
{
    CONTROL_Q2_IDLE = 0,
    CONTROL_Q2_AB_STRAIGHT,
    CONTROL_Q2_FIND_B_LINE,
    CONTROL_Q2_BC_ARC,
    CONTROL_Q2_C_ALIGN,
    CONTROL_Q2_CD_STRAIGHT,
    CONTROL_Q2_FIND_D_LINE,
    CONTROL_Q2_DA_ARC
} ControlQ2Phase;

typedef enum
{
    CONTROL_Q3_IDLE = 0,
    CONTROL_Q3_START_ALIGN,
    CONTROL_Q3_AC_STRAIGHT,
    CONTROL_Q3_FIND_C_LINE,
    CONTROL_Q3_CB_ARC,
    CONTROL_Q3_B_ALIGN,
    CONTROL_Q3_BD_STRAIGHT,
    CONTROL_Q3_FIND_D_LINE,
    CONTROL_Q3_DA_ARC
} ControlQ3Phase;

int16_t g_control_task_base_speed;
int16_t g_control_task_turn;
volatile UBaseType_t g_debug_control_stack_high_water_mark;
/* Runtime line-loop tuning: 4 means 0.004 encoder-count turn per error unit. */
volatile int32_t g_line_follow_kp_x1000 = 4;
volatile int32_t g_line_follow_kd_x1000 = 0;
volatile int8_t g_line_follow_steering_sign = -1;
volatile int32_t g_line_exit_target_degrees = 18;
volatile int32_t g_line_exit_track_width_mm = 140;
volatile int32_t g_line_exit_angle_mdeg;
volatile int16_t g_line_exit_turn;
volatile int32_t g_q1_target_distance_mm = 900;
volatile int32_t g_q1_cruise_rpm = 120;
volatile int32_t g_q1_line_search_rpm = 60;
volatile int32_t g_q1_line_search_window_mm = 80;
volatile int32_t g_q1_stop_overrun_mm = 60;
volatile uint8_t g_q1_line_confirm_cycles = 2u;
volatile int32_t g_q1_travel_mm;
volatile int32_t g_q1_heading_error_mm;
/* Encoder heading loop for unmarked straights.  One millimetre of wheel
 * travel mismatch now produces one delta-count of correction.  The small
 * positive trim raises the left-wheel target and removes the chassis'
 * repeatable left drift. */
volatile int32_t g_straight_sync_kp_x1000 = 1000;
volatile int16_t g_straight_sync_max_turn = 6;
volatile int16_t g_straight_trim_delta = 1;
volatile int16_t g_debug_straight_sync_turn;
volatile int32_t g_q2_straight_distance_mm = 1000;
volatile int32_t g_q2_cd_slow_window_mm = 250;
volatile int32_t g_q2_cd_slow_rpm = 60;
volatile int32_t g_q2_cd_search_rpm = 40;
/* Radius is 400 mm, so a semicircle is about 1257 mm. */
volatile int32_t g_q2_arc_min_distance_mm = 1050;
volatile uint8_t g_q2_arc_lost_confirm_cycles = 3u;
/* Arm endpoint detection only after the line has been tracked stably. */
volatile uint8_t g_arc_endpoint_arm_cycles = 10u;
volatile int32_t g_q2_arc_entry_rpm = 60;
volatile int32_t g_q2_arc_cruise_rpm = 100;
volatile int32_t g_q2_arc_entry_distance_mm = 180;
volatile int32_t g_q2_arc_recovery_rpm = 60;
volatile uint8_t g_q2_arc_recovery_stop_cycles = 40u;
volatile uint8_t g_q2_arc_recovery_lost_cycles;
volatile int32_t g_q2_c_extra_turn_degrees = 24;
/* Pivot speed used for the fixed heading correction after point C. */
volatile int32_t g_q2_c_align_rpm = 70;
/* +1 continues the clockwise/right B-C semicircle at C; set -1 if the
 * physical chassis turns in the opposite direction. */
volatile int8_t g_q2_c_align_turn_sign = -1;
volatile int32_t g_q2_c_align_angle_mdeg;
volatile uint8_t g_q2_phase;
volatile int32_t g_q2_segment_travel_mm;
volatile int32_t g_q2_arc_travel_mm;
/* Q3: A-C and B-D are sqrt(1000^2 + 800^2), approximately 1280 mm. */
volatile int32_t g_q3_diagonal_distance_mm = 1280;
volatile int32_t g_q3_line_search_window_mm = 180;
volatile int32_t g_q3_bd_slow_window_mm = 350;
volatile int32_t g_q3_bd_slow_rpm = 50;
volatile int32_t g_q3_bd_search_rpm = 30;
/* With the chassis initially pointing from A toward B, rotate right to face C. */
volatile int32_t g_q3_start_align_degrees = 43;
volatile int32_t g_q3_start_align_rpm = 60;
volatile int8_t g_q3_start_align_sign = -1;
/* Smooth B-to-D transition: keep moving while changing heading. */
volatile int32_t g_q3_transition_forward_rpm = 60;
volatile int32_t g_q3_align_rpm = 50;
volatile int32_t g_q3_b_align_degrees = 41;
volatile int8_t g_q3_b_align_sign = 1;
volatile uint8_t g_q3_phase;
volatile int32_t g_q3_segment_travel_mm;
volatile int32_t g_q3_arc_travel_mm;
/* Failsafe beyond the nominal 1257 mm semicircle. */
volatile int32_t g_q3_arc_hard_stop_mm = 1450;
volatile int32_t g_q3_align_angle_mdeg;
volatile uint8_t g_q4_completed_laps;
volatile int32_t g_q4_initial_align_degrees = 40;
volatile int32_t g_q4_a_restart_degrees = 90;
volatile int8_t g_q4_a_align_sign = -1;
volatile int32_t g_q4_repeat_ac_rpm = 80;
/* Q4 slows before the ends of both 1257 mm semicircles. */
volatile int32_t g_q4_arc_slow_start_mm = 1150;
volatile int32_t g_q4_point_approach_rpm = 60;
volatile int32_t g_q4_b_slow_start_mm = 1150;
volatile int32_t g_q4_b_approach_rpm = 50;
volatile int8_t g_control_task_corner_state;
volatile int8_t g_control_task_corner_hint;
volatile int32_t g_control_task_filtered_line_error;
volatile int32_t g_control_task_line_error_derivative;
volatile int32_t g_control_task_corner_entry_distance_mm;

static ControlCornerDirection detect_corner_hint(uint8_t active_mask)
{
    uint8_t left_seen = active_mask & CONTROL_TASK_LEFT_BRANCH_MASK;
    uint8_t right_seen = active_mask & CONTROL_TASK_RIGHT_BRANCH_MASK;

    if ((left_seen != 0u) && (right_seen == 0u))
    {
#if BOARD_LINE_SENSOR_INDEX0_IS_LEFT
        return CONTROL_CORNER_LEFT;
#else
        return CONTROL_CORNER_RIGHT;
#endif
    }
    if ((right_seen != 0u) && (left_seen == 0u))
    {
#if BOARD_LINE_SENSOR_INDEX0_IS_LEFT
        return CONTROL_CORNER_RIGHT;
#else
        return CONTROL_CORNER_LEFT;
#endif
    }

    return CONTROL_CORNER_NONE;
}

static int16_t rpm_to_encoder_delta(int32_t rpm)
{
    int64_t numerator;

    if (rpm <= 0)
    {
        return 1;
    }
    if (rpm > (int32_t)BOARD_CHASSIS_RATED_SPEED_RPM)
    {
        rpm = (int32_t)BOARD_CHASSIS_RATED_SPEED_RPM;
    }

    numerator = (int64_t)BOARD_ENCODER_OUTPUT_PULSES_PER_REV * rpm *
                BOARD_CONTROL_TASK_PERIOD_MS;
    return (int16_t)((numerator + 30000) / 60000);
}

static int16_t straight_sync_turn(int32_t left_travel_mm,
                                  int32_t right_travel_mm)
{
    int32_t travel_error_mm = left_travel_mm - right_travel_mm;
    int32_t turn = -((travel_error_mm * g_straight_sync_kp_x1000) / 1000) +
                   g_straight_trim_delta;
    int32_t limit = g_straight_sync_max_turn;

    if (limit < 0)
        limit = -limit;
    if (turn > limit)
        turn = limit;
    else if (turn < -limit)
        turn = -limit;

    g_debug_straight_sync_turn = (int16_t)turn;
    return (int16_t)turn;
}

void StartcontrolTask(void *argument)
{
    TickType_t wake_tick = xTaskGetTickCount();
    LowPassFilter line_error_filter;
    ControlCornerDirection corner_state = CONTROL_CORNER_NONE;
    ControlCornerDirection corner_hint = CONTROL_CORNER_NONE;
    uint16_t corner_hint_hold = 0u;
    uint16_t corner_cycles = 0u;
    uint16_t corner_turn_cycles = 0u;
    uint8_t corner_reacquire_count = 0u;
    uint8_t corner_turn_started = 0u;
    uint8_t line_seen_since_run = 0u;
    uint8_t exit_align_active = 0u;
    int32_t corner_entry_left_mm = 0;
    int32_t corner_entry_right_mm = 0;
    int32_t exit_entry_left_mm = 0;
    int32_t exit_entry_right_mm = 0;
    int32_t q1_entry_left_mm = 0;
    int32_t q1_entry_right_mm = 0;
    uint8_t q1_active = 0u;
    uint8_t q1_line_found_count = 0u;
    ControlQ2Phase q2_phase = CONTROL_Q2_IDLE;
    uint8_t q2_active = 0u;
    uint8_t q2_arc_lost_count = 0u;
    uint8_t q2_arc_tracking_count = 0u;
    uint8_t q2_arc_recovery_lost_count = 0u;
    uint8_t q2_line_found_count = 0u;
    uint8_t q2_c_align_settle_cycles = 0u;
    int32_t q2_segment_left_mm = 0;
    int32_t q2_segment_right_mm = 0;
    int32_t q2_arc_left_mm = 0;
    int32_t q2_arc_right_mm = 0;
    int32_t q2_align_left_mm = 0;
    int32_t q2_align_right_mm = 0;
    ControlQ3Phase q3_phase = CONTROL_Q3_IDLE;
    uint8_t q3_active = 0u;
    uint8_t q3_line_found_count = 0u;
    uint8_t q3_arc_lost_count = 0u;
    uint8_t q3_arc_tracking_count = 0u;
    uint8_t q3_arc_recovery_lost_count = 0u;
    uint8_t q3_straight_settle_cycles = 0u;
    uint8_t q4_completed_laps = 0u;
    int32_t q3_segment_left_mm = 0;
    int32_t q3_segment_right_mm = 0;
    int32_t q3_arc_left_mm = 0;
    int32_t q3_arc_right_mm = 0;
    int32_t q3_align_left_mm = 0;
    int32_t q3_align_right_mm = 0;
    int32_t last_filtered_error = 0;
    uint8_t start_delay_state = 0u;
    TickType_t start_delay_tick = 0u;

    (void)argument;
    LowPassFilter_Init(&line_error_filter, CONTROL_TASK_LINE_FILTER_DIV);

    for (;;)
    {
        AppLineFollow_Snapshot line_snapshot;
        int32_t filtered_error;
        int32_t error_derivative;
        int32_t speed_reduction;
        int16_t target_turn;
        int16_t base_speed;

        g_debug_control_stack_high_water_mark =
            uxTaskGetStackHighWaterMark(NULL);

        if (AppRaceConfig_IsRunning() == 0u)
        {
            AppRaceConfig_Snapshot race_snapshot = AppRaceConfig_GetSnapshot();

            g_control_task_turn = 0;
            g_control_task_corner_state = CONTROL_CORNER_NONE;
            g_control_task_corner_hint = CONTROL_CORNER_NONE;
            corner_state = CONTROL_CORNER_NONE;
            corner_hint = CONTROL_CORNER_NONE;
            corner_hint_hold = 0u;
            corner_cycles = 0u;
            corner_turn_cycles = 0u;
            corner_reacquire_count = 0u;
            corner_turn_started = 0u;
            line_seen_since_run = 0u;
            exit_align_active = 0u;
            /* STOP is a pause: retain route entries and phase so RUN resumes
             * from the same physical segment. READY/DONE starts a new route. */
            if (race_snapshot.state != APP_RACE_STATE_STOPPED)
            {
                q1_active = 0u;
                q1_line_found_count = 0u;
                q2_active = 0u;
                q2_phase = CONTROL_Q2_IDLE;
                q2_arc_lost_count = 0u;
                q2_arc_tracking_count = 0u;
                q2_arc_recovery_lost_count = 0u;
                g_q2_arc_recovery_lost_cycles = 0u;
                q2_line_found_count = 0u;
                q2_c_align_settle_cycles = 0u;
                g_q2_phase = CONTROL_Q2_IDLE;
                g_q2_segment_travel_mm = 0;
                g_q2_arc_travel_mm = 0;

                q3_active = 0u;
                q3_phase = CONTROL_Q3_IDLE;
                q3_line_found_count = 0u;
                q3_arc_lost_count = 0u;
                q3_arc_tracking_count = 0u;
                q3_arc_recovery_lost_count = 0u;
                q3_straight_settle_cycles = 0u;
                g_q3_phase = CONTROL_Q3_IDLE;
                g_q3_segment_travel_mm = 0;
                g_q3_arc_travel_mm = 0;
                g_q3_align_angle_mdeg = 0;
                q4_completed_laps = 0u;
                g_q4_completed_laps = 0u;
            }
            g_line_exit_angle_mdeg = 0;
            g_line_exit_turn = 0;
            last_filtered_error = 0;
            g_control_task_filtered_line_error = 0;
            g_control_task_line_error_derivative = 0;
            g_control_task_corner_entry_distance_mm = 0;
            LowPassFilter_Reset(&line_error_filter);
            AppChassis_Stop();
            start_delay_state = 0u;
            vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
            continue;
        }

        /* Non-blocking 500 ms launch delay.  Other tasks, OLED and keys keep
         * running, and STOP can still cancel the pending launch. */
        if (start_delay_state == 0u)
        {
            start_delay_state = 1u;
            start_delay_tick = xTaskGetTickCount();
        }
        if (start_delay_state == 1u)
        {
            if ((xTaskGetTickCount() - start_delay_tick) <
                pdMS_TO_TICKS(CONTROL_TASK_START_DELAY_MS))
            {
                AppChassis_Stop();
                vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                continue;
            }
            start_delay_state = 2u;
        }

#if BOARD_DIRECT_300_RPM_TEST
        /* Pure speed-loop verification. Use SetTargetSpeed so the command is
         * applied immediately instead of passing through the route slew and
         * line-error speed reduction. */
        g_control_task_turn = 0;
        AppChassis_SetTargetSpeed((int16_t)BOARD_CHASSIS_RATED_SPEED_DELTA,
                                  (int16_t)BOARD_CHASSIS_RATED_SPEED_DELTA);
        vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
        continue;
#endif

        /* Q1: A -> B is an unmarked 100 cm straight. Use encoder distance
         * instead of line sensors, and correct heading from wheel travel
         * difference. */
        if (AppRaceConfig_GetMode() == 1u)
        {
            AppChassis_Snapshot chassis_snapshot = AppChassis_GetSnapshot();
            int32_t left_travel;
            int32_t right_travel;
            int32_t average_travel;
            int32_t heading_error;
            int32_t remaining;
            int32_t search_start_mm;
            int32_t hard_stop_mm;
            int16_t q1_speed;
            uint8_t point_line_valid;

            line_snapshot = AppLineFollow_GetSnapshot();

            if (q1_active == 0u)
            {
                q1_active = 1u;
                q1_entry_left_mm = chassis_snapshot.left_encoder_mm;
                q1_entry_right_mm = chassis_snapshot.right_encoder_mm;
                g_q1_travel_mm = 0;
                g_q1_heading_error_mm = 0;
                q1_line_found_count = 0u;
            }

            left_travel = chassis_snapshot.left_encoder_mm - q1_entry_left_mm;
            right_travel = chassis_snapshot.right_encoder_mm - q1_entry_right_mm;
            if (left_travel < 0)
            {
                left_travel = -left_travel;
            }
            if (right_travel < 0)
            {
                right_travel = -right_travel;
            }
            average_travel = (left_travel + right_travel) / 2;
            heading_error = left_travel - right_travel;
            g_q1_travel_mm = average_travel;
            g_q1_heading_error_mm = heading_error;

            search_start_mm = g_q1_target_distance_mm -
                              g_q1_line_search_window_mm;
            if (search_start_mm < 0)
                search_start_mm = 0;
            hard_stop_mm = g_q1_target_distance_mm + g_q1_stop_overrun_mm;
            point_line_valid =
                (line_snapshot.status == LINE_TRACKER_OK) &&
                ((line_snapshot.active_mask & CONTROL_POINT_CENTER_MASK) != 0u) &&
                (line_snapshot.error >= -CONTROL_POINT_MAX_ERROR) &&
                (line_snapshot.error <= CONTROL_POINT_MAX_ERROR);

            if ((average_travel >= search_start_mm) &&
                (point_line_valid != 0u))
            {
                if (q1_line_found_count < g_q1_line_confirm_cycles)
                    ++q1_line_found_count;
            }
            else
            {
                q1_line_found_count = 0u;
            }

            if ((g_q1_target_distance_mm <= 0) ||
                ((g_q1_line_confirm_cycles > 0u) &&
                 (q1_line_found_count >= g_q1_line_confirm_cycles)) ||
                (average_travel >= hard_stop_mm))
            {
                AppChassis_Brake();
                AppRaceConfig_SetFinished((uint32_t)xTaskGetTickCount());
                q1_active = 0u;
                q1_line_found_count = 0u;
                vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                continue;
            }

            q1_speed = rpm_to_encoder_delta(g_q1_cruise_rpm);
            remaining = g_q1_target_distance_mm - average_travel;
            if (remaining <= g_q1_line_search_window_mm)
            {
                q1_speed = rpm_to_encoder_delta(g_q1_line_search_rpm);
            }
            if (q1_speed < 1)
            {
                q1_speed = 1;
            }

            /* Do not make a late heading correction while the front sensor is
             * already searching for B; at low speed even a small turn causes
             * a visible sideways hook before braking. */
            g_control_task_turn =
                (remaining <= g_q1_line_search_window_mm) ? 0 : straight_sync_turn(left_travel, right_travel);
            AppChassis_SetTargetVelocity(q1_speed, g_control_task_turn);
            vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
            continue;
        }
        q1_active = 0u;

        /* Q2: encoder-controlled A-B and C-D straights, with line-followed
         * B-C and D-A semicircles. Each boundary point produces one prompt. */
        if (AppRaceConfig_GetMode() == 2u)
        {
            AppChassis_Snapshot chassis_snapshot = AppChassis_GetSnapshot();
            /* Q1 and Q2 share the same RPM setting. */
            int16_t q2_speed = rpm_to_encoder_delta(g_q1_cruise_rpm);

            line_snapshot = AppLineFollow_GetSnapshot();
            if (q2_active == 0u)
            {
                q2_active = 1u;
                q2_phase = CONTROL_Q2_AB_STRAIGHT;
                q2_segment_left_mm = chassis_snapshot.left_encoder_mm;
                q2_segment_right_mm = chassis_snapshot.right_encoder_mm;
                q2_arc_lost_count = 0u;
                q2_arc_tracking_count = 0u;
                q2_arc_recovery_lost_count = 0u;
                g_q2_arc_recovery_lost_cycles = 0u;
                q2_line_found_count = 0u;
                g_q2_segment_travel_mm = 0;
                g_q2_arc_travel_mm = 0;
            }
            g_q2_phase = (uint8_t)q2_phase;

            if ((q2_phase == CONTROL_Q2_AB_STRAIGHT) ||
                (q2_phase == CONTROL_Q2_CD_STRAIGHT))
            {
                int32_t left_travel = chassis_snapshot.left_encoder_mm - q2_segment_left_mm;
                int32_t right_travel = chassis_snapshot.right_encoder_mm - q2_segment_right_mm;
                int32_t average_travel;
                int32_t remaining;

                /* The pivot-to-straight transition must not be interpreted as
                 * a new heading error.  Drive both wheels at exactly the same
                 * target while the pivot motion settles, then re-zero only
                 * the left/right difference while retaining mean distance. */
                if ((q2_phase == CONTROL_Q2_CD_STRAIGHT) &&
                    (q2_c_align_settle_cycles > 0u))
                {
                    int32_t settled_average = (left_travel + right_travel) / 2;

                    AppChassis_SetTargetSpeed(q2_speed, q2_speed);
                    --q2_c_align_settle_cycles;
                    if (q2_c_align_settle_cycles == 0u)
                    {
                        q2_segment_left_mm =
                            chassis_snapshot.left_encoder_mm - settled_average;
                        q2_segment_right_mm =
                            chassis_snapshot.right_encoder_mm - settled_average;
                    }
                    g_control_task_turn = 0;
                    g_debug_straight_sync_turn = 0;
                    g_q2_segment_travel_mm =
                        (settled_average < 0) ? -settled_average : settled_average;
                    vTaskDelayUntil(&wake_tick,
                                    pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                    continue;
                }

                if (left_travel < 0)
                    left_travel = -left_travel;
                if (right_travel < 0)
                    right_travel = -right_travel;
                average_travel = (left_travel + right_travel) / 2;
                g_q2_segment_travel_mm = average_travel;

                if ((g_q2_straight_distance_mm <= 0) ||
                    (average_travel >= g_q2_straight_distance_mm))
                {
                    q2_phase = (q2_phase == CONTROL_Q2_AB_STRAIGHT) ? CONTROL_Q2_FIND_B_LINE : CONTROL_Q2_FIND_D_LINE;
                    q2_line_found_count = 0u;
                    g_q2_phase = (uint8_t)q2_phase;
                    g_control_task_turn = 0;
                    if (q2_phase == CONTROL_Q2_FIND_D_LINE)
                        q2_speed = rpm_to_encoder_delta(g_q2_cd_search_rpm);
                    else
                        q2_speed = (int16_t)(q2_speed / 2);
                    AppChassis_SetTargetVelocity(q2_speed, 0);
                    vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                    continue;
                }

                remaining = g_q2_straight_distance_mm - average_travel;
                if ((q2_phase == CONTROL_Q2_CD_STRAIGHT) &&
                    (remaining <= g_q2_cd_slow_window_mm))
                {
                    q2_speed = rpm_to_encoder_delta(
                        (remaining < 60) ? g_q2_cd_search_rpm
                                         : g_q2_cd_slow_rpm);
                }
                else if (remaining < 50)
                    q2_speed = (int16_t)(q2_speed / 3);
                else if (remaining < 150)
                    q2_speed = (int16_t)((q2_speed * 2) / 3);
                if (q2_speed < 1)
                    q2_speed = 1;

                g_control_task_turn = straight_sync_turn(left_travel, right_travel);
                AppChassis_SetTargetVelocity(q2_speed, g_control_task_turn);
                vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                continue;
            }

            if ((q2_phase == CONTROL_Q2_FIND_B_LINE) ||
                (q2_phase == CONTROL_Q2_FIND_D_LINE))
            {
                uint8_t point_line_valid =
                    (line_snapshot.status == LINE_TRACKER_OK) &&
                    ((line_snapshot.active_mask & CONTROL_POINT_CENTER_MASK) != 0u) &&
                    (line_snapshot.error >= -CONTROL_POINT_MAX_ERROR) &&
                    (line_snapshot.error <= CONTROL_POINT_MAX_ERROR);

                if (point_line_valid != 0u)
                {
                    if (q2_line_found_count < CONTROL_POINT_CONFIRM_CYCLES)
                        ++q2_line_found_count;
                }
                else
                {
                    q2_line_found_count = 0u;
                }

                if (q2_line_found_count >= CONTROL_POINT_CONFIRM_CYCLES)
                {
                    /* B/D is reached only when the following arc is actually
                     * under the sensor, not merely when odometry reaches 1 m. */
                    AppRaceConfig_NotifyPoint((uint32_t)xTaskGetTickCount());
                    q2_phase = (q2_phase == CONTROL_Q2_FIND_B_LINE) ? CONTROL_Q2_BC_ARC : CONTROL_Q2_DA_ARC;
                    q2_arc_left_mm = chassis_snapshot.left_encoder_mm;
                    q2_arc_right_mm = chassis_snapshot.right_encoder_mm;
                    q2_arc_lost_count = 0u;
                    q2_arc_tracking_count = 0u;
                    q2_arc_recovery_lost_count = 0u;
                    g_q2_arc_recovery_lost_cycles = 0u;
                    q2_line_found_count = 0u;
                    line_seen_since_run = 1u;
                    g_q2_arc_travel_mm = 0;
                    g_q2_phase = (uint8_t)q2_phase;
                    LowPassFilter_Reset(&line_error_filter);
                    last_filtered_error = 0;
                }
                else
                {
                    g_control_task_turn = 0;
                    if (q2_phase == CONTROL_Q2_FIND_D_LINE)
                        q2_speed = rpm_to_encoder_delta(g_q2_cd_search_rpm);
                    else
                        q2_speed = (int16_t)(q2_speed / 2);
                    AppChassis_SetTargetVelocity(q2_speed, 0);
                    vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                    continue;
                }
            }

            if ((q2_phase == CONTROL_Q2_BC_ARC) ||
                (q2_phase == CONTROL_Q2_DA_ARC))
            {
                int32_t left_travel = chassis_snapshot.left_encoder_mm - q2_arc_left_mm;
                int32_t right_travel = chassis_snapshot.right_encoder_mm - q2_arc_right_mm;
                int32_t average_travel;

                if (left_travel < 0)
                    left_travel = -left_travel;
                if (right_travel < 0)
                    right_travel = -right_travel;
                average_travel = (left_travel + right_travel) / 2;
                g_q2_arc_travel_mm = average_travel;

                if ((line_snapshot.status == LINE_TRACKER_OK) &&
                    (line_snapshot.active_mask != 0u) &&
                    (q2_arc_tracking_count < g_arc_endpoint_arm_cycles))
                {
                    ++q2_arc_tracking_count;
                }

                if ((q2_arc_tracking_count >= g_arc_endpoint_arm_cycles) &&
                    (line_snapshot.active_mask == 0u))
                {
                    if (q2_arc_lost_count < 255u)
                        ++q2_arc_lost_count;
                }
                else
                {
                    q2_arc_lost_count = 0u;
                }

                if (line_snapshot.status == LINE_TRACKER_OK)
                {
                    q2_arc_recovery_lost_count = 0u;
                    g_q2_arc_recovery_lost_cycles = 0u;
                }

                if (q2_arc_lost_count >= g_q2_arc_lost_confirm_cycles)
                {
                    q2_arc_lost_count = 0u;
                    g_control_task_turn = 0;
                    if (q2_phase == CONTROL_Q2_DA_ARC)
                    {
                        /* Reaching A is also the finish notification; do not
                         * start a second overlapping point beep. */
                        AppChassis_Brake();
                        AppRaceConfig_SetFinished((uint32_t)xTaskGetTickCount());
                        q2_active = 0u;
                        q2_phase = CONTROL_Q2_IDLE;
                        g_q2_phase = CONTROL_Q2_IDLE;
                    }
                    else
                    {
                        AppRaceConfig_NotifyPoint((uint32_t)xTaskGetTickCount());
                        q2_phase = CONTROL_Q2_C_ALIGN;
                        q2_align_left_mm = chassis_snapshot.left_encoder_mm;
                        q2_align_right_mm = chassis_snapshot.right_encoder_mm;
                        g_line_exit_angle_mdeg = 0;
                        g_q2_c_align_angle_mdeg = 0;
                        g_q2_phase = (uint8_t)q2_phase;
                        /* Start the C-point heading correction immediately.
                         * A zero-forward-speed pivot gives both wheels enough
                         * drive and avoids appearing to stop at point C. */
                        AppChassis_SetTargetVelocity(
                            0,
                            (int16_t)(((g_q2_c_align_turn_sign < 0) ? -1 : 1) *
                                      rpm_to_encoder_delta(g_q2_c_align_rpm)));
                    }
                    vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                    continue;
                }

                if (line_snapshot.status != LINE_TRACKER_OK)
                {
                    int16_t recovery_speed =
                        rpm_to_encoder_delta(g_q2_arc_recovery_rpm);

                    if (q2_arc_recovery_lost_count < 255u)
                        ++q2_arc_recovery_lost_count;
                    g_q2_arc_recovery_lost_cycles =
                        q2_arc_recovery_lost_count;

                    if ((g_q2_arc_recovery_stop_cycles > 0u) &&
                        (q2_arc_recovery_lost_count >=
                         g_q2_arc_recovery_stop_cycles))
                    {
                        /* Do not keep driving blind after a failed recovery.
                         * A valid line sample automatically releases braking. */
                        AppChassis_Brake();
                    }
                    else
                    {
                        AppChassis_SetTargetVelocity(recovery_speed,
                                                     g_control_task_turn);
                    }
                    vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                    continue;
                }
            }

            if (q2_phase == CONTROL_Q2_C_ALIGN)
            {
                int32_t left_travel = chassis_snapshot.left_encoder_mm - q2_align_left_mm;
                int32_t right_travel = chassis_snapshot.right_encoder_mm - q2_align_right_mm;
                int32_t difference;
                int32_t track_width = g_line_exit_track_width_mm;
                int32_t target_angle_mdeg;
                int32_t turn_sign = (g_q2_c_align_turn_sign < 0) ? -1 : 1;
                int16_t align_turn;

                /* The requested direction is fixed by turn_sign.  Use the
                 * absolute encoder difference for progress so reversed
                 * encoder polarity cannot leave the state stuck at C. */
                difference = left_travel - right_travel;
                if (difference < 0)
                    difference = -difference;
                if (track_width < 1)
                    track_width = 1;
                g_q2_c_align_angle_mdeg =
                    (int32_t)(((int64_t)difference * 57296) / track_width);
                g_line_exit_angle_mdeg = g_q2_c_align_angle_mdeg;
                target_angle_mdeg = g_q2_c_extra_turn_degrees * 1000;

                if ((target_angle_mdeg <= 0) ||
                    (g_q2_c_align_angle_mdeg >= target_angle_mdeg))
                {
                    q2_phase = CONTROL_Q2_CD_STRAIGHT;
                    q2_segment_left_mm = chassis_snapshot.left_encoder_mm;
                    q2_segment_right_mm = chassis_snapshot.right_encoder_mm;
                    g_q2_segment_travel_mm = 0;
                    g_q2_phase = (uint8_t)q2_phase;
                    g_control_task_turn = 0;
                    q2_c_align_settle_cycles =
                        CONTROL_Q2_C_ALIGN_SETTLE_CYCLES;
                    /* Bypass velocity slew here.  Otherwise the old pivot
                     * targets persist into C-D and the heading loop turns the
                     * chassis back to cancel them. */
                    AppChassis_SetTargetSpeed(q2_speed, q2_speed);
                }
                else
                {
                    align_turn = (int16_t)(turn_sign *
                                           rpm_to_encoder_delta(g_q2_c_align_rpm));
                    g_line_exit_turn = align_turn;
                    g_control_task_turn = align_turn;
                    AppChassis_SetTargetVelocity(0, align_turn);
                }
                vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                continue;
            }
        }
        else
        {
            q2_active = 0u;
            q2_phase = CONTROL_Q2_IDLE;
            q2_line_found_count = 0u;
            q2_arc_recovery_lost_count = 0u;
            g_q2_arc_recovery_lost_cycles = 0u;
            g_q2_phase = CONTROL_Q2_IDLE;
        }

        /* Q3: A-C diagonal, C-B right semicircle, B-D diagonal, then D-A
         * left semicircle.  Encoder odometry handles the two unmarked
         * diagonals; the seven-way grayscale board confirms C/D and follows
         * both arcs. */
        if ((AppRaceConfig_GetMode() == 3u) ||
            (AppRaceConfig_GetMode() == 4u))
        {
            AppChassis_Snapshot chassis_snapshot = AppChassis_GetSnapshot();
            int16_t q3_speed = rpm_to_encoder_delta(g_q1_cruise_rpm);

            line_snapshot = AppLineFollow_GetSnapshot();
            if (q3_active == 0u)
            {
                q3_active = 1u;
                q3_phase = CONTROL_Q3_START_ALIGN;
                q3_align_left_mm = chassis_snapshot.left_encoder_mm;
                q3_align_right_mm = chassis_snapshot.right_encoder_mm;
                q3_line_found_count = 0u;
                q3_arc_lost_count = 0u;
                q3_arc_tracking_count = 0u;
                q3_arc_recovery_lost_count = 0u;
                q3_straight_settle_cycles = 0u;
                g_q3_segment_travel_mm = 0;
                g_q3_arc_travel_mm = 0;
                g_q3_align_angle_mdeg = 0;
            }
            g_q3_phase = (uint8_t)q3_phase;

            /* The first Q4 lap keeps Q3's normal speed.  Laps 2-4 leave A
             * more gently along the A-C diagonal. */
            if ((AppRaceConfig_GetMode() == 4u) &&
                (q4_completed_laps > 0u) &&
                ((q3_phase == CONTROL_Q3_START_ALIGN) ||
                 (q3_phase == CONTROL_Q3_AC_STRAIGHT)))
            {
                q3_speed = rpm_to_encoder_delta(g_q4_repeat_ac_rpm);
            }

            if (q3_phase == CONTROL_Q3_START_ALIGN)
            {
                int32_t left_travel =
                    chassis_snapshot.left_encoder_mm - q3_align_left_mm;
                int32_t right_travel =
                    chassis_snapshot.right_encoder_mm - q3_align_right_mm;
                int32_t difference = left_travel - right_travel;
                int32_t track_width = g_line_exit_track_width_mm;
                int32_t turn_sign =
                    (g_q3_start_align_sign < 0) ? -1 : 1;
                int32_t start_target_degrees = g_q3_start_align_degrees;
                int16_t align_turn;

                if (AppRaceConfig_GetMode() == 4u)
                {
                    turn_sign = (g_q4_a_align_sign < 0) ? -1 : 1;
                    start_target_degrees =
                        (q4_completed_laps == 0u)
                            ? g_q4_initial_align_degrees
                            : g_q4_a_restart_degrees;
                }

                if (difference < 0)
                    difference = -difference;
                if (track_width < 1)
                    track_width = 1;
                g_q3_align_angle_mdeg =
                    (int32_t)(((int64_t)difference * 57296) / track_width);

                if ((start_target_degrees <= 0) ||
                    (g_q3_align_angle_mdeg >=
                     (start_target_degrees * 1000)))
                {
                    q3_phase = CONTROL_Q3_AC_STRAIGHT;
                    q3_segment_left_mm = chassis_snapshot.left_encoder_mm;
                    q3_segment_right_mm = chassis_snapshot.right_encoder_mm;
                    q3_straight_settle_cycles =
                        CONTROL_Q2_C_ALIGN_SETTLE_CYCLES;
                    g_q3_segment_travel_mm = 0;
                    g_q3_phase = (uint8_t)q3_phase;
                    g_control_task_turn = 0;
                    AppChassis_SetTargetSpeed(q3_speed, q3_speed);
                }
                else
                {
                    align_turn = (int16_t)(turn_sign *
                                           rpm_to_encoder_delta(
                                               g_q3_start_align_rpm));
                    g_control_task_turn = align_turn;
                    AppChassis_SetTargetVelocity(0, align_turn);
                }
                vTaskDelayUntil(&wake_tick,
                                pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                continue;
            }

            if ((q3_phase == CONTROL_Q3_AC_STRAIGHT) ||
                (q3_phase == CONTROL_Q3_BD_STRAIGHT))
            {
                int32_t left_travel =
                    chassis_snapshot.left_encoder_mm - q3_segment_left_mm;
                int32_t right_travel =
                    chassis_snapshot.right_encoder_mm - q3_segment_right_mm;
                int32_t average_travel;
                int32_t remaining;
                int32_t search_start_mm;
                uint8_t point_line_valid;

                if (q3_straight_settle_cycles > 0u)
                {
                    int32_t settled_average = (left_travel + right_travel) / 2;

                    AppChassis_SetTargetSpeed(q3_speed, q3_speed);
                    --q3_straight_settle_cycles;
                    if (q3_straight_settle_cycles == 0u)
                    {
                        q3_segment_left_mm =
                            chassis_snapshot.left_encoder_mm - settled_average;
                        q3_segment_right_mm =
                            chassis_snapshot.right_encoder_mm - settled_average;
                    }
                    g_control_task_turn = 0;
                    g_debug_straight_sync_turn = 0;
                    g_q3_segment_travel_mm =
                        (settled_average < 0) ? -settled_average : settled_average;
                    vTaskDelayUntil(&wake_tick,
                                    pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                    continue;
                }

                if (left_travel < 0)
                    left_travel = -left_travel;
                if (right_travel < 0)
                    right_travel = -right_travel;
                average_travel = (left_travel + right_travel) / 2;
                g_q3_segment_travel_mm = average_travel;

                search_start_mm = g_q3_diagonal_distance_mm -
                                  g_q3_line_search_window_mm;
                if (search_start_mm < 0)
                    search_start_mm = 0;
                point_line_valid =
                    (line_snapshot.status == LINE_TRACKER_OK) &&
                    ((line_snapshot.active_mask & CONTROL_POINT_CENTER_MASK) != 0u) &&
                    (line_snapshot.error >= -CONTROL_POINT_MAX_ERROR) &&
                    (line_snapshot.error <= CONTROL_POINT_MAX_ERROR);
                if (q3_phase == CONTROL_Q3_BD_STRAIGHT)
                {
                    /* The diagonal can meet D with the black line under an
                     * outer sensor first.  Accept any stable black detection
                     * here instead of requiring the center three sensors. */
                    point_line_valid =
                        (line_snapshot.status != LINE_TRACKER_LOST) &&
                        (line_snapshot.active_mask != 0u);
                }
                if ((average_travel >= search_start_mm) &&
                    (point_line_valid != 0u))
                {
                    if (q3_line_found_count < CONTROL_POINT_CONFIRM_CYCLES)
                        ++q3_line_found_count;
                }
                else
                {
                    q3_line_found_count = 0u;
                }

                if (q3_line_found_count >= CONTROL_POINT_CONFIRM_CYCLES)
                {
                    AppRaceConfig_NotifyPoint((uint32_t)xTaskGetTickCount());
                    if (q3_phase == CONTROL_Q3_AC_STRAIGHT)
                    {
                        /* C is confirmed by the grayscale board.  Enter arc
                         * tracking directly; do not impose an encoder-angle
                         * pivot before the line follower gets control. */
                        q3_phase = CONTROL_Q3_CB_ARC;
                        q3_arc_left_mm = chassis_snapshot.left_encoder_mm;
                        q3_arc_right_mm = chassis_snapshot.right_encoder_mm;
                        q3_arc_lost_count = 0u;
                        q3_arc_tracking_count = 0u;
                        q3_arc_recovery_lost_count = 0u;
                        g_q3_arc_travel_mm = 0;
                        line_seen_since_run = 1u;
                        LowPassFilter_Reset(&line_error_filter);
                        last_filtered_error = 0;
                    }
                    else
                    {
                        q3_phase = CONTROL_Q3_DA_ARC;
                        q3_arc_left_mm = chassis_snapshot.left_encoder_mm;
                        q3_arc_right_mm = chassis_snapshot.right_encoder_mm;
                        q3_arc_lost_count = 0u;
                        q3_arc_tracking_count = 0u;
                        q3_arc_recovery_lost_count = 0u;
                        g_q3_arc_travel_mm = 0;
                        line_seen_since_run = 1u;
                        LowPassFilter_Reset(&line_error_filter);
                        last_filtered_error = 0;
                    }
                    q3_line_found_count = 0u;
                    g_q3_phase = (uint8_t)q3_phase;
                    if ((q3_phase != CONTROL_Q3_CB_ARC) &&
                        (q3_phase != CONTROL_Q3_DA_ARC))
                    {
                        AppChassis_Stop();
                    }
                    else
                    {
                        AppChassis_SetTargetVelocity(
                            rpm_to_encoder_delta(g_q2_arc_entry_rpm), 0);
                    }
                    vTaskDelayUntil(&wake_tick,
                                    pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                    continue;
                }

                if ((g_q3_diagonal_distance_mm <= 0) ||
                    (average_travel >= g_q3_diagonal_distance_mm))
                {
                    q3_phase = (q3_phase == CONTROL_Q3_AC_STRAIGHT)
                                   ? CONTROL_Q3_FIND_C_LINE
                                   : CONTROL_Q3_FIND_D_LINE;
                    q3_line_found_count = 0u;
                    g_q3_phase = (uint8_t)q3_phase;
                    g_control_task_turn = 0;
                    if (q3_phase == CONTROL_Q3_FIND_D_LINE)
                        q3_speed = rpm_to_encoder_delta(g_q3_bd_search_rpm);
                    else
                        q3_speed = (int16_t)(q3_speed / 2);
                    AppChassis_SetTargetVelocity(q3_speed, 0);
                    vTaskDelayUntil(&wake_tick,
                                    pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                    continue;
                }

                remaining = g_q3_diagonal_distance_mm - average_travel;
                if (((q3_phase == CONTROL_Q3_BD_STRAIGHT) ||
                     (AppRaceConfig_GetMode() == 4u)) &&
                    (remaining <= g_q3_bd_slow_window_mm))
                {
                    q3_speed = rpm_to_encoder_delta(
                        (remaining < 60) ? g_q3_bd_search_rpm
                                         : g_q3_bd_slow_rpm);
                }
                else if (remaining < 50)
                    q3_speed = (int16_t)(q3_speed / 3);
                else if (remaining < 150)
                    q3_speed = (int16_t)((q3_speed * 2) / 3);
                if (q3_speed < 1)
                    q3_speed = 1;

                g_control_task_turn =
                    straight_sync_turn(left_travel, right_travel);
                AppChassis_SetTargetVelocity(q3_speed, g_control_task_turn);
                vTaskDelayUntil(&wake_tick,
                                pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                continue;
            }

            if ((q3_phase == CONTROL_Q3_FIND_C_LINE) ||
                (q3_phase == CONTROL_Q3_FIND_D_LINE))
            {
                uint8_t point_line_valid =
                    (line_snapshot.status == LINE_TRACKER_OK) &&
                    ((line_snapshot.active_mask & CONTROL_POINT_CENTER_MASK) != 0u) &&
                    (line_snapshot.error >= -CONTROL_POINT_MAX_ERROR) &&
                    (line_snapshot.error <= CONTROL_POINT_MAX_ERROR);

                if (q3_phase == CONTROL_Q3_FIND_D_LINE)
                {
                    point_line_valid =
                        (line_snapshot.status != LINE_TRACKER_LOST) &&
                        (line_snapshot.active_mask != 0u);
                }

                if (point_line_valid != 0u)
                {
                    if (q3_line_found_count < CONTROL_POINT_CONFIRM_CYCLES)
                        ++q3_line_found_count;
                }
                else
                {
                    q3_line_found_count = 0u;
                }

                if (q3_line_found_count >= CONTROL_POINT_CONFIRM_CYCLES)
                {
                    AppRaceConfig_NotifyPoint((uint32_t)xTaskGetTickCount());
                    if (q3_phase == CONTROL_Q3_FIND_C_LINE)
                    {
                        q3_phase = CONTROL_Q3_CB_ARC;
                        q3_arc_left_mm = chassis_snapshot.left_encoder_mm;
                        q3_arc_right_mm = chassis_snapshot.right_encoder_mm;
                        q3_arc_lost_count = 0u;
                        q3_arc_tracking_count = 0u;
                        q3_arc_recovery_lost_count = 0u;
                        g_q3_arc_travel_mm = 0;
                        line_seen_since_run = 1u;
                        LowPassFilter_Reset(&line_error_filter);
                        last_filtered_error = 0;
                    }
                    else
                    {
                        q3_phase = CONTROL_Q3_DA_ARC;
                        q3_arc_left_mm = chassis_snapshot.left_encoder_mm;
                        q3_arc_right_mm = chassis_snapshot.right_encoder_mm;
                        q3_arc_lost_count = 0u;
                        q3_arc_tracking_count = 0u;
                        q3_arc_recovery_lost_count = 0u;
                        g_q3_arc_travel_mm = 0;
                        line_seen_since_run = 1u;
                        LowPassFilter_Reset(&line_error_filter);
                        last_filtered_error = 0;
                    }
                    q3_line_found_count = 0u;
                    g_q3_phase = (uint8_t)q3_phase;
                }
                else
                {
                    g_control_task_turn = 0;
                    if (q3_phase == CONTROL_Q3_FIND_D_LINE)
                        q3_speed = rpm_to_encoder_delta(g_q3_bd_search_rpm);
                    else
                        q3_speed = (int16_t)(q3_speed / 2);
                    AppChassis_SetTargetVelocity(q3_speed, 0);
                    vTaskDelayUntil(&wake_tick,
                                    pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                    continue;
                }
            }

            if (q3_phase == CONTROL_Q3_B_ALIGN)
            {
                int32_t left_travel =
                    chassis_snapshot.left_encoder_mm - q3_align_left_mm;
                int32_t right_travel =
                    chassis_snapshot.right_encoder_mm - q3_align_right_mm;
                int32_t difference = left_travel - right_travel;
                int32_t track_width = g_line_exit_track_width_mm;
                int32_t target_degrees;
                int32_t turn_sign;
                int16_t align_turn;

                if (difference < 0)
                    difference = -difference;
                if (track_width < 1)
                    track_width = 1;
                g_q3_align_angle_mdeg =
                    (int32_t)(((int64_t)difference * 57296) / track_width);

                target_degrees = g_q3_b_align_degrees;
                turn_sign = (g_q3_b_align_sign < 0) ? -1 : 1;

                if ((target_degrees <= 0) ||
                    (g_q3_align_angle_mdeg >= (target_degrees * 1000)))
                {
                    q3_arc_lost_count = 0u;
                    q3_arc_tracking_count = 0u;
                    q3_arc_recovery_lost_count = 0u;
                    LowPassFilter_Reset(&line_error_filter);
                    last_filtered_error = 0;
                    line_seen_since_run = 1u;

                    q3_phase = CONTROL_Q3_BD_STRAIGHT;
                    q3_segment_left_mm = chassis_snapshot.left_encoder_mm;
                    q3_segment_right_mm = chassis_snapshot.right_encoder_mm;
                    q3_straight_settle_cycles =
                        CONTROL_Q2_C_ALIGN_SETTLE_CYCLES;
                    g_q3_segment_travel_mm = 0;
                    AppChassis_SetTargetSpeed(q3_speed, q3_speed);
                    g_q3_phase = (uint8_t)q3_phase;
                }
                else
                {
                    align_turn = (int16_t)(turn_sign *
                                           rpm_to_encoder_delta(g_q3_align_rpm));
                    g_control_task_turn = align_turn;
                    AppChassis_SetTargetVelocity(
                        rpm_to_encoder_delta(g_q3_transition_forward_rpm),
                        align_turn);
                }
                vTaskDelayUntil(&wake_tick,
                                pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                continue;
            }

            if ((q3_phase == CONTROL_Q3_CB_ARC) ||
                (q3_phase == CONTROL_Q3_DA_ARC))
            {
                int32_t left_travel =
                    chassis_snapshot.left_encoder_mm - q3_arc_left_mm;
                int32_t right_travel =
                    chassis_snapshot.right_encoder_mm - q3_arc_right_mm;
                int32_t average_travel;

                if (left_travel < 0)
                    left_travel = -left_travel;
                if (right_travel < 0)
                    right_travel = -right_travel;
                average_travel = (left_travel + right_travel) / 2;
                g_q3_arc_travel_mm = average_travel;

                if ((line_snapshot.status == LINE_TRACKER_OK) &&
                    (line_snapshot.active_mask != 0u) &&
                    (q3_arc_tracking_count < g_arc_endpoint_arm_cycles))
                {
                    ++q3_arc_tracking_count;
                }

                if ((q3_arc_tracking_count >= g_arc_endpoint_arm_cycles) &&
                    (line_snapshot.active_mask == 0u))
                {
                    if (q3_arc_lost_count < 255u)
                        ++q3_arc_lost_count;
                }
                else
                {
                    q3_arc_lost_count = 0u;
                }

                if (line_snapshot.status == LINE_TRACKER_OK)
                    q3_arc_recovery_lost_count = 0u;

                if ((q3_arc_lost_count >= g_q2_arc_lost_confirm_cycles) ||
                    ((g_q3_arc_hard_stop_mm > 0) &&
                     (average_travel >= g_q3_arc_hard_stop_mm)))
                {
                    q3_arc_lost_count = 0u;
                    g_control_task_turn = 0;
                    if (q3_phase == CONTROL_Q3_DA_ARC)
                    {
                        if (AppRaceConfig_GetMode() == 4u)
                        {
                            if (q4_completed_laps < 4u)
                                ++q4_completed_laps;
                            g_q4_completed_laps = q4_completed_laps;
                            AppRaceConfig_SetLapProgress(q4_completed_laps, 4u);

                            if (q4_completed_laps >= 4u)
                            {
                                AppChassis_Brake();
                                AppRaceConfig_SetFinished(
                                    (uint32_t)xTaskGetTickCount());
                                q3_active = 0u;
                                q3_phase = CONTROL_Q3_IDLE;
                            }
                            else
                            {
                                /* At A the arc tangent already points toward
                                 * B.  Announce the lap, then reuse Q3's
                                 * automatic initial turn for the next A-C. */
                                AppRaceConfig_NotifyPoint(
                                    (uint32_t)xTaskGetTickCount());
                                q3_phase = CONTROL_Q3_START_ALIGN;
                                q3_align_left_mm =
                                    chassis_snapshot.left_encoder_mm;
                                q3_align_right_mm =
                                    chassis_snapshot.right_encoder_mm;
                                g_q3_align_angle_mdeg = 0;
                                q3_arc_tracking_count = 0u;
                                q3_arc_recovery_lost_count = 0u;
                                AppChassis_Stop();
                            }
                        }
                        else
                        {
                            AppChassis_Brake();
                            AppRaceConfig_SetFinished(
                                (uint32_t)xTaskGetTickCount());
                            q3_active = 0u;
                            q3_phase = CONTROL_Q3_IDLE;
                        }
                    }
                    else
                    {
                        AppRaceConfig_NotifyPoint(
                            (uint32_t)xTaskGetTickCount());
                        q3_phase = CONTROL_Q3_B_ALIGN;
                        q3_align_left_mm = chassis_snapshot.left_encoder_mm;
                        q3_align_right_mm = chassis_snapshot.right_encoder_mm;
                        g_q3_align_angle_mdeg = 0;
                    }
                    g_q3_phase = (uint8_t)q3_phase;
                    vTaskDelayUntil(&wake_tick,
                                    pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                    continue;
                }

                if (line_snapshot.status != LINE_TRACKER_OK)
                {
                    int16_t recovery_speed =
                        rpm_to_encoder_delta(g_q2_arc_recovery_rpm);

                    if (q3_arc_recovery_lost_count < 255u)
                        ++q3_arc_recovery_lost_count;
                    if ((g_q2_arc_recovery_stop_cycles > 0u) &&
                        (q3_arc_recovery_lost_count >=
                         g_q2_arc_recovery_stop_cycles))
                    {
                        AppChassis_Brake();
                    }
                    else
                    {
                        AppChassis_SetTargetVelocity(recovery_speed,
                                                     g_control_task_turn);
                    }
                    vTaskDelayUntil(&wake_tick,
                                    pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                    continue;
                }
            }
        }
        else
        {
            q3_active = 0u;
            q3_phase = CONTROL_Q3_IDLE;
            q3_line_found_count = 0u;
            q3_arc_lost_count = 0u;
            q3_arc_tracking_count = 0u;
            q3_arc_recovery_lost_count = 0u;
            q3_straight_settle_cycles = 0u;
            g_q3_phase = CONTROL_Q3_IDLE;
        }

        line_snapshot = AppLineFollow_GetSnapshot();

        if (corner_state != CONTROL_CORNER_NONE)
        {
            AppChassis_Snapshot chassis_snapshot = AppChassis_GetSnapshot();
            base_speed = (g_control_task_base_speed != 0) ? g_control_task_base_speed : CONTROL_TASK_DEFAULT_BASE_SPEED;
            ++corner_cycles;

            /* Advance by encoder distance after losing the line. Unlike a fixed
             * delay, this reaches the same physical turn point as speed and battery
             * voltage change. */
            if (!corner_turn_started)
            {
                int32_t left_distance = chassis_snapshot.left_encoder_mm - corner_entry_left_mm;
                int32_t right_distance = chassis_snapshot.right_encoder_mm - corner_entry_right_mm;
                int32_t average_distance;
                int16_t entry_speed =
                    (int16_t)(base_speed / CONTROL_TASK_CORNER_ENTRY_SPEED_DIV);

                if (left_distance < 0)
                    left_distance = -left_distance;
                if (right_distance < 0)
                    right_distance = -right_distance;
                average_distance = (left_distance + right_distance) / 2;
                g_control_task_corner_entry_distance_mm = average_distance;
                if (average_distance < CONTROL_TASK_CORNER_ENTRY_DISTANCE_MM)
                {
                    g_control_task_turn = 0;
                    AppChassis_SetTargetSpeed(entry_speed, entry_speed);
                    vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                    continue;
                }

                corner_turn_started = 1u;
                corner_turn_cycles = 0u;
            }
            ++corner_turn_cycles;

            if ((corner_turn_cycles >= CONTROL_TASK_CORNER_MIN_TURN_CYCLES) &&
                (line_snapshot.status == LINE_TRACKER_OK) &&
                ((line_snapshot.active_mask & CONTROL_TASK_CENTER_REACQUIRE_MASK) != 0u))
            {
                ++corner_reacquire_count;
            }
            else
            {
                corner_reacquire_count = 0u;
            }

            if (corner_reacquire_count >= CONTROL_TASK_CORNER_REACQUIRE_CYCLES)
            {
                corner_state = CONTROL_CORNER_NONE;
                corner_hint = CONTROL_CORNER_NONE;
                corner_hint_hold = 0u;
                corner_cycles = 0u;
                corner_turn_cycles = 0u;
                corner_reacquire_count = 0u;
                corner_turn_started = 0u;
                g_control_task_corner_state = CONTROL_CORNER_NONE;
                g_control_task_corner_hint = CONTROL_CORNER_NONE;
                g_control_task_turn = 0;
                last_filtered_error = 0;
                g_control_task_filtered_line_error = 0;
                g_control_task_line_error_derivative = 0;
                LowPassFilter_Reset(&line_error_filter);
                AppChassis_SetTargetSpeed(
                    (int16_t)(base_speed / CONTROL_TASK_CORNER_ENTRY_SPEED_DIV),
                    (int16_t)(base_speed / CONTROL_TASK_CORNER_ENTRY_SPEED_DIV));
            }
            else if (corner_cycles >= CONTROL_TASK_CORNER_TIMEOUT_CYCLES)
            {
                corner_state = CONTROL_CORNER_NONE;
                corner_hint = CONTROL_CORNER_NONE;
                corner_hint_hold = 0u;
                corner_cycles = 0u;
                corner_turn_cycles = 0u;
                corner_reacquire_count = 0u;
                corner_turn_started = 0u;
                g_control_task_corner_state = CONTROL_CORNER_NONE;
                g_control_task_corner_hint = CONTROL_CORNER_NONE;
                g_control_task_turn = 0;
                last_filtered_error = 0;
                g_control_task_filtered_line_error = 0;
                g_control_task_line_error_derivative = 0;
                AppChassis_Stop();
            }
            else if (corner_state == CONTROL_CORNER_LEFT)
            {
                int16_t turn_speed =
                    (int16_t)(base_speed / CONTROL_TASK_CORNER_TURN_SPEED_DIV);

                g_control_task_turn = (int16_t)-turn_speed;
                AppChassis_SetTargetSpeed(0, turn_speed);
            }
            else
            {
                int16_t turn_speed =
                    (int16_t)(base_speed / CONTROL_TASK_CORNER_TURN_SPEED_DIV);

                g_control_task_turn = turn_speed;
                AppChassis_SetTargetSpeed(turn_speed, 0);
            }

            vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
            continue;
        }

        if (line_snapshot.status == LINE_TRACKER_OK)
        {
            ControlCornerDirection detected_hint = detect_corner_hint(line_snapshot.active_mask);

            if (detected_hint != CONTROL_CORNER_NONE)
            {
                corner_hint = detected_hint;
                corner_hint_hold = CONTROL_TASK_CORNER_HINT_HOLD_CYCLES;
                g_control_task_corner_hint = (int8_t)corner_hint;
            }
            else if (corner_hint_hold > 0u)
            {
                --corner_hint_hold;
                if (corner_hint_hold == 0u)
                {
                    corner_hint = CONTROL_CORNER_NONE;
                    g_control_task_corner_hint = CONTROL_CORNER_NONE;
                }
            }
        }

        /* Start a remembered right-angle turn only after the current black
         * line has completely left all seven sensors. Use active_mask so this
         * remains correct for either electrical polarity. */
        if ((CONTROL_TASK_ENABLE_CORNER_RECOVERY != 0u) &&
            (line_snapshot.active_mask == 0u) &&
            (corner_hint != CONTROL_CORNER_NONE) && (corner_hint_hold > 0u))
        {
            AppChassis_Snapshot chassis_snapshot = AppChassis_GetSnapshot();
            corner_state = corner_hint;
            corner_cycles = 0u;
            corner_turn_cycles = 0u;
            corner_reacquire_count = 0u;
            corner_turn_started = 0u;
            corner_entry_left_mm = chassis_snapshot.left_encoder_mm;
            corner_entry_right_mm = chassis_snapshot.right_encoder_mm;
            g_control_task_corner_entry_distance_mm = 0;
            g_control_task_corner_state = (int8_t)corner_state;
            last_filtered_error = 0;
            g_control_task_filtered_line_error = 0;
            g_control_task_line_error_derivative = 0;
            LowPassFilter_Reset(&line_error_filter);
            vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
            continue;
        }

        if (line_snapshot.status != LINE_TRACKER_OK)
        {
            AppChassis_Snapshot chassis_snapshot;
            int32_t left_distance;
            int32_t right_distance;
            int32_t distance_difference;
            int32_t track_width;
            int16_t exit_speed;
            int16_t exit_turn;

            if (line_seen_since_run == 0u)
            {
                AppChassis_Stop();
                vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                continue;
            }

            base_speed = (g_control_task_base_speed != 0) ? g_control_task_base_speed : CONTROL_TASK_DEFAULT_BASE_SPEED;
            chassis_snapshot = AppChassis_GetSnapshot();
            if (exit_align_active == 0u)
            {
                exit_align_active = 1u;
                exit_entry_left_mm = chassis_snapshot.left_encoder_mm;
                exit_entry_right_mm = chassis_snapshot.right_encoder_mm;
                g_line_exit_angle_mdeg = 0;
            }

            left_distance = chassis_snapshot.left_encoder_mm - exit_entry_left_mm;
            right_distance = chassis_snapshot.right_encoder_mm - exit_entry_right_mm;
            if (left_distance < 0)
            {
                left_distance = -left_distance;
            }
            if (right_distance < 0)
            {
                right_distance = -right_distance;
            }
            distance_difference = left_distance - right_distance;
            if (distance_difference < 0)
            {
                distance_difference = -distance_difference;
            }
            track_width = g_line_exit_track_width_mm;
            if (track_width < 1)
            {
                track_width = 1;
            }
            /* yaw[mdeg] = wheel-distance difference / track width * 180/pi */
            g_line_exit_angle_mdeg =
                (int32_t)(((int64_t)distance_difference * 57296) / track_width);

            if ((g_line_exit_target_degrees <= 0) ||
                (g_line_exit_angle_mdeg >= (g_line_exit_target_degrees * 1000)))
            {
                g_control_task_turn = 0;
                g_line_exit_turn = 0;
                AppChassis_Stop();
                vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
                continue;
            }

            /* After leaving the arc, apply a measured left-heading correction
             * and stop when the encoder-derived target angle is reached. */
            exit_speed = (int16_t)((base_speed * CONTROL_TASK_EXIT_ALIGN_SPEED_NUM) /
                                   CONTROL_TASK_EXIT_ALIGN_SPEED_DIV);
            exit_turn = (int16_t)((exit_speed * CONTROL_TASK_MAX_TURN_NUM) /
                                  CONTROL_TASK_MAX_TURN_DIV);
            g_line_exit_turn = exit_turn;
            g_control_task_turn = exit_turn;
            g_control_task_line_error_derivative = 0;
            AppChassis_SetTargetVelocity(exit_speed, exit_turn);
            vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
            continue;
        }
        line_seen_since_run = 1u;
        exit_align_active = 0u;
        g_line_exit_angle_mdeg = 0;
        g_line_exit_turn = 0;

        if ((AppRaceConfig_GetMode() == 2u) &&
            ((q2_phase == CONTROL_Q2_BC_ARC) ||
             (q2_phase == CONTROL_Q2_DA_ARC)))
        {
            base_speed = rpm_to_encoder_delta(
                (g_q2_arc_travel_mm < g_q2_arc_entry_distance_mm) ? g_q2_arc_entry_rpm : g_q2_arc_cruise_rpm);
        }
        else if (((AppRaceConfig_GetMode() == 3u) ||
                  (AppRaceConfig_GetMode() == 4u)) &&
                 ((q3_phase == CONTROL_Q3_CB_ARC) ||
                  (q3_phase == CONTROL_Q3_DA_ARC)))
        {
            if ((AppRaceConfig_GetMode() == 4u) &&
                (q3_phase == CONTROL_Q3_CB_ARC) &&
                (g_q3_arc_travel_mm >= g_q4_b_slow_start_mm))
            {
                base_speed =
                    rpm_to_encoder_delta(g_q4_b_approach_rpm);
            }
            else if ((AppRaceConfig_GetMode() == 4u) &&
                     (g_q3_arc_travel_mm >= g_q4_arc_slow_start_mm))
            {
                base_speed =
                    rpm_to_encoder_delta(g_q4_point_approach_rpm);
            }
            else
            {
                base_speed = rpm_to_encoder_delta(
                    (g_q3_arc_travel_mm < g_q2_arc_entry_distance_mm)
                        ? g_q2_arc_entry_rpm
                        : g_q2_arc_cruise_rpm);
            }
        }
        else
        {
            base_speed = (g_control_task_base_speed != 0) ? g_control_task_base_speed : CONTROL_TASK_DEFAULT_BASE_SPEED;
        }
        filtered_error = LowPassFilter_Update(&line_error_filter, line_snapshot.error);
        error_derivative = filtered_error - last_filtered_error;
        last_filtered_error = filtered_error;
        g_control_task_filtered_line_error = filtered_error;
        g_control_task_line_error_derivative = error_derivative;
        if ((filtered_error >= -CONTROL_TASK_LINE_ERROR_DEADBAND) &&
            (filtered_error <= CONTROL_TASK_LINE_ERROR_DEADBAND))
        {
            target_turn = 0;
        }
        else
        {
            int32_t turn_value =
                ((filtered_error * g_line_follow_kp_x1000) +
                 (error_derivative * g_line_follow_kd_x1000)) /
                CONTROL_TASK_LINE_GAIN_SCALE;

            if (g_line_follow_steering_sign < 0)
            {
                turn_value = -turn_value;
            }
            if (turn_value > APP_CHASSIS_MAX_SPEED)
            {
                turn_value = APP_CHASSIS_MAX_SPEED;
            }
            else if (turn_value < -APP_CHASSIS_MAX_SPEED)
            {
                turn_value = -APP_CHASSIS_MAX_SPEED;
            }
            target_turn = (int16_t)turn_value;
        }

        /* Once the line reaches an outermost sensor, use a decisive pivot
         * correction. This guarantees a real wheel-speed difference instead
         * of trying to recover at nearly full straight-line speed. */
        if ((line_snapshot.active_mask & 0x01u) != 0u)
        {
#if BOARD_LINE_SENSOR_INDEX0_IS_LEFT
            target_turn = base_speed;
#else
            target_turn = (int16_t)-base_speed;
#endif
        }
        else if ((line_snapshot.active_mask & 0x40u) != 0u)
        {
#if BOARD_LINE_SENSOR_INDEX0_IS_LEFT
            target_turn = (int16_t)-base_speed;
#else
            target_turn = base_speed;
#endif
        }

        if (target_turn > (int16_t)(g_control_task_turn + CONTROL_TASK_TURN_SLEW_STEP))
        {
            g_control_task_turn = (int16_t)(g_control_task_turn + CONTROL_TASK_TURN_SLEW_STEP);
        }
        else if (target_turn < (int16_t)(g_control_task_turn - CONTROL_TASK_TURN_SLEW_STEP))
        {
            g_control_task_turn = (int16_t)(g_control_task_turn - CONTROL_TASK_TURN_SLEW_STEP);
        }
        else
        {
            g_control_task_turn = target_turn;
        }

        /* Keep line following from commanding one wheel in reverse. */
        if (g_control_task_turn > base_speed)
        {
            g_control_task_turn = base_speed;
        }
        else if (g_control_task_turn < -base_speed)
        {
            g_control_task_turn = (int16_t)-base_speed;
        }

        speed_reduction = (filtered_error < 0 ? -filtered_error : filtered_error) /
                          CONTROL_TASK_SPEED_REDUCTION_DIV;
        if (speed_reduction > (base_speed / CONTROL_TASK_MIN_SPEED_DIV))
        {
            speed_reduction = base_speed / CONTROL_TASK_MIN_SPEED_DIV;
        }
        base_speed = (int16_t)(base_speed - speed_reduction);
        {
            int16_t max_turn = (int16_t)((base_speed * CONTROL_TASK_MAX_TURN_NUM) /
                                         CONTROL_TASK_MAX_TURN_DIV);

            if (max_turn < 1)
            {
                max_turn = 1;
            }
            if (g_control_task_turn > max_turn)
            {
                g_control_task_turn = max_turn;
            }
            else if (g_control_task_turn < -max_turn)
            {
                g_control_task_turn = (int16_t)-max_turn;
            }
        }

        /* Q4 exits both semicircles tangentially.  In the final section near
         * B/A, keep the reduced speed but stop applying line steering so the
         * chassis leaves the arc with a straight heading. */
        if ((AppRaceConfig_GetMode() == 4u) &&
            (((q3_phase == CONTROL_Q3_CB_ARC) &&
              (g_q3_arc_travel_mm >= g_q4_b_slow_start_mm)) ||
             ((q3_phase == CONTROL_Q3_DA_ARC) &&
              (g_q3_arc_travel_mm >= g_q4_arc_slow_start_mm))))
        {
            g_control_task_turn = 0;
            g_line_exit_turn = 0;
        }

        AppChassis_SetTargetVelocity(base_speed, g_control_task_turn);

        vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}
