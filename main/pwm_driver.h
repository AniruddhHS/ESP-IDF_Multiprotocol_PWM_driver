/*
 * pwm_driver.h
 *
 *  Created on: 30-Sep-2023
 *      Author: aniru
 */

#ifndef MAIN_PWM_DRIVER_H_
#define MAIN_PWM_DRIVER_H_

#include <stdio.h>
#include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "config.h"

// LEDCPWM
#if PWM_TYPE == 1
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE

#define LEDC_HS_TIMER0          LEDC_TIMER_0
#define LEDC_HS_CH0_GPIO       M1_GPIO
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0

#define LEDC_HS_TIMER1          LEDC_TIMER_1
#define LEDC_HS_CH1_GPIO       M2_GPIO
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1

#define LEDC_HS_TIMER2          LEDC_TIMER_2
#define LEDC_HS_CH2_GPIO       M3_GPIO
#define LEDC_HS_CH2_CHANNEL    LEDC_CHANNEL_2

#define LEDC_HS_TIMER3          LEDC_TIMER_3
#define LEDC_HS_CH3_GPIO       M4_GPIO
#define LEDC_HS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_CH_NUM       (4)
#endif

// MCPWM
#if PWM_TYPE == 2


#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define SERVO_1_PULSE_GPIO             M1_GPIO        // GPIO connects to the PWM signal line

#define SERVO_2_PULSE_GPIO             M2_GPIO       // GPIO connects to the PWM signal line

#define SERVO_3_PULSE_GPIO             M3_GPIO        // GPIO connects to the PWM signal line

#define SERVO_4_PULSE_GPIO             M4_GPIO        // GPIO connects to the PWM signal line

#endif

// DSHOT
#if PWM_TYPE == 3
#endif

void pwm_init();
void pwm_write(uint16_t channel, float duty);

#endif /* MAIN_PWM_DRIVER_H_ */
