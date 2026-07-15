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
#include "cmsis_os.h"
#include "lib_rate_filter.h"

#define CONTROL_TASK_PERIOD_MS BOARD_CONTROL_TASK_PERIOD_MS
#define CONTROL_TASK_LINE_KP_DIV 500
#define CONTROL_TASK_LINE_KD_DIV 1000
#define CONTROL_TASK_DEFAULT_BASE_SPEED ((int16_t)BOARD_CHASSIS_DEFAULT_SPEED_DELTA)
#define CONTROL_TASK_LINE_FILTER_DIV 2u
#define CONTROL_TASK_LINE_ERROR_DEADBAND 600
#define CONTROL_TASK_TURN_SLEW_STEP 2
#define CONTROL_TASK_SPEED_REDUCTION_DIV 500
#define CONTROL_TASK_MIN_SPEED_DIV 2
#define CONTROL_TASK_SENSOR_NO_LINE_RAW_MASK 0x00u
#define CONTROL_TASK_LEFT_BRANCH_MASK 0x03u
#define CONTROL_TASK_RIGHT_BRANCH_MASK 0x60u
#define CONTROL_TASK_CENTER_REACQUIRE_MASK 0x1Cu
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

int16_t g_control_task_base_speed;
int16_t g_control_task_turn;
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

void StartcontrolTask(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();
    LowPassFilter line_error_filter;
    ControlCornerDirection corner_state = CONTROL_CORNER_NONE;
    ControlCornerDirection corner_hint = CONTROL_CORNER_NONE;
    uint16_t corner_hint_hold = 0u;
    uint16_t corner_cycles = 0u;
	uint16_t corner_turn_cycles = 0u;
    uint8_t corner_reacquire_count = 0u;
	uint8_t corner_turn_started = 0u;
	int32_t corner_entry_left_mm = 0;
	int32_t corner_entry_right_mm = 0;
    int32_t last_filtered_error = 0;

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

        if (AppRaceConfig_IsRunning() == 0u)
        {
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
            last_filtered_error = 0;
            g_control_task_filtered_line_error = 0;
            g_control_task_line_error_derivative = 0;
			g_control_task_corner_entry_distance_mm = 0;
            LowPassFilter_Reset(&line_error_filter);
            AppChassis_Stop();
            wake_tick += CONTROL_TASK_PERIOD_MS;
            osDelayUntil(wake_tick);
            continue;
        }

        line_snapshot = AppLineFollow_GetSnapshot();

        if (corner_state != CONTROL_CORNER_NONE)
        {
			AppChassis_Snapshot chassis_snapshot = AppChassis_GetSnapshot();
            base_speed = (g_control_task_base_speed != 0) ?
                         g_control_task_base_speed : CONTROL_TASK_DEFAULT_BASE_SPEED;
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
					wake_tick += CONTROL_TASK_PERIOD_MS;
					osDelayUntil(wake_tick);
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

            wake_tick += CONTROL_TASK_PERIOD_MS;
            osDelayUntil(wake_tick);
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

        /* With the verified board polarity, white floor is raw 0x00. Start a
         * remembered right-angle turn only
         * after the current black line has completely left all seven sensors. */
        if ((line_snapshot.raw_mask == CONTROL_TASK_SENSOR_NO_LINE_RAW_MASK) &&
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
            wake_tick += CONTROL_TASK_PERIOD_MS;
            osDelayUntil(wake_tick);
            continue;
        }

        if (line_snapshot.status != LINE_TRACKER_OK)
        {
            /*
             * No line (LOST) and all sensors active (FULL) are ambiguous.
             * Do not feed either condition into the speed loop: its integral
             * term would otherwise keep driving the motors toward saturation.
             */
            g_control_task_turn = 0;
			last_filtered_error = 0;
			g_control_task_filtered_line_error = 0;
			g_control_task_line_error_derivative = 0;
            LowPassFilter_Reset(&line_error_filter);
            AppChassis_Stop();
            wake_tick += CONTROL_TASK_PERIOD_MS;
            osDelayUntil(wake_tick);
            continue;
        }

        base_speed = (g_control_task_base_speed != 0) ? g_control_task_base_speed : CONTROL_TASK_DEFAULT_BASE_SPEED;
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
            target_turn = (int16_t)((filtered_error / CONTROL_TASK_LINE_KP_DIV) +
                                    (error_derivative / CONTROL_TASK_LINE_KD_DIV));
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
		if (g_control_task_turn > base_speed)
		{
			g_control_task_turn = base_speed;
		}
		else if (g_control_task_turn < -base_speed)
		{
			g_control_task_turn = (int16_t)-base_speed;
		}

        AppChassis_SetTargetSpeed((int16_t)(base_speed + g_control_task_turn),
                                  (int16_t)(base_speed - g_control_task_turn));

        wake_tick += CONTROL_TASK_PERIOD_MS;
        osDelayUntil(wake_tick);
    }
}
