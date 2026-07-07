/**
 * @file board_config.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief
 * @version 1.0
 * @date 2026-07-07
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Keys: one side connects to GPIO, the other side connects to GND. */
#define BOARD_KEY_DEFAULT_ACTIVE_LOW       1u
#define BOARD_KEY_DEFAULT_DEBOUNCE_MS      20u
#define BOARD_KEY_DEFAULT_LONG_PRESS_MS    1000u

#define BOARD_KEY_PA8_GPIO_PORT            GPIOA
#define BOARD_KEY_PA8_GPIO_PIN             GPIO_PIN_8
#define BOARD_KEY_PC13_GPIO_PORT           GPIOC
#define BOARD_KEY_PC13_GPIO_PIN            GPIO_PIN_13
#define BOARD_KEY_PC14_GPIO_PORT           GPIOC
#define BOARD_KEY_PC14_GPIO_PIN            GPIO_PIN_14

/* Buzzer */
#define BOARD_BUZZER_GPIO_PORT             GPIOB
#define BOARD_BUZZER_GPIO_PIN              GPIO_PIN_5
#define BOARD_BUZZER_DEFAULT_ACTIVE_LOW    0u

/* LED */
#define BOARD_LED_GPIO_PORT                GPIOC
#define BOARD_LED_GPIO_PIN                 GPIO_PIN_4
#define BOARD_LED_DEFAULT_ACTIVE_LOW       0u

/* Line sensors */
#define BOARD_LINE_SENSOR_DEFAULT_ACTIVE_LOW 0u
#define BOARD_LINE_SENSOR_0_GPIO_PORT      GPIOC
#define BOARD_LINE_SENSOR_0_GPIO_PIN       GPIO_PIN_0
#define BOARD_LINE_SENSOR_1_GPIO_PORT      GPIOC
#define BOARD_LINE_SENSOR_1_GPIO_PIN       GPIO_PIN_1
#define BOARD_LINE_SENSOR_2_GPIO_PORT      GPIOC
#define BOARD_LINE_SENSOR_2_GPIO_PIN       GPIO_PIN_2
#define BOARD_LINE_SENSOR_3_GPIO_PORT      GPIOC
#define BOARD_LINE_SENSOR_3_GPIO_PIN       GPIO_PIN_3
#define BOARD_LINE_SENSOR_4_GPIO_PORT      GPIOC
#define BOARD_LINE_SENSOR_4_GPIO_PIN       GPIO_PIN_5
#define BOARD_LINE_SENSOR_5_GPIO_PORT      GPIOC
#define BOARD_LINE_SENSOR_5_GPIO_PIN       GPIO_PIN_8
#define BOARD_LINE_SENSOR_6_GPIO_PORT      GPIOC
#define BOARD_LINE_SENSOR_6_GPIO_PIN       GPIO_PIN_9
#define BOARD_LINE_SENSOR_7_GPIO_PORT      GPIOC
#define BOARD_LINE_SENSOR_7_GPIO_PIN       GPIO_PIN_12

/* TB6612 motor driver */
#define BOARD_TB6612_PWM_MAX_DUTY          1000u

#define BOARD_TB6612_LEFT_IN1_GPIO_PORT    GPIOA
#define BOARD_TB6612_LEFT_IN1_GPIO_PIN     GPIO_PIN_5
#define BOARD_TB6612_LEFT_IN2_GPIO_PORT    GPIOA
#define BOARD_TB6612_LEFT_IN2_GPIO_PIN     GPIO_PIN_4
#define BOARD_TB6612_LEFT_PWM_CHANNEL      TIM_CHANNEL_1

#define BOARD_TB6612_RIGHT_IN1_GPIO_PORT   GPIOB
#define BOARD_TB6612_RIGHT_IN1_GPIO_PIN    GPIO_PIN_0
#define BOARD_TB6612_RIGHT_IN2_GPIO_PORT   GPIOB
#define BOARD_TB6612_RIGHT_IN2_GPIO_PIN    GPIO_PIN_1
#define BOARD_TB6612_RIGHT_PWM_CHANNEL     TIM_CHANNEL_2

/*
 * Encoder
 * Measured at gearbox output shaft. If one physical output revolution reads
 * 1456 counts in software, change this value from 364u to 1456u.
 */
#define BOARD_ENCODER_OUTPUT_PULSES_PER_REV 364u
#define BOARD_ENCODER_LEFT_INVERTED         0u
#define BOARD_ENCODER_RIGHT_INVERTED        0u

/* Wheel */
#define BOARD_WHEEL_DIAMETER_MM             65u
#define BOARD_WHEEL_CIRCUMFERENCE_UM        ((BOARD_WHEEL_DIAMETER_MM * 31416u) / 10u)

/* Chassis line-follow default speed, in encoder pulses per control period. */
#define BOARD_CONTROL_TASK_PERIOD_MS        10u
#define BOARD_CHASSIS_DEFAULT_SPEED_RPM     300u
#define BOARD_CHASSIS_DEFAULT_SPEED_DELTA   \
    ((BOARD_ENCODER_OUTPUT_PULSES_PER_REV * BOARD_CHASSIS_DEFAULT_SPEED_RPM * \
      BOARD_CONTROL_TASK_PERIOD_MS) / 60000u)

/* Speed PID. Target and feedback are encoder pulses per control period. */
#define BOARD_CHASSIS_SPEED_PID_KP          30
#define BOARD_CHASSIS_SPEED_PID_KI          2
#define BOARD_CHASSIS_SPEED_PID_KD          0
#define BOARD_CHASSIS_SPEED_PID_SCALE       1
#define BOARD_CHASSIS_SPEED_PID_INTEGRAL_MIN (-300)
#define BOARD_CHASSIS_SPEED_PID_INTEGRAL_MAX 300
#define BOARD_CHASSIS_SPEED_PID_OUTPUT_MIN  (-APP_CHASSIS_MAX_SPEED)
#define BOARD_CHASSIS_SPEED_PID_OUTPUT_MAX  APP_CHASSIS_MAX_SPEED

#ifdef __cplusplus
}
#endif

#endif
