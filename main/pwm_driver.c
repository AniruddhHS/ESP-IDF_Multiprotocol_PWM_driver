/*
 * pwm_driver.c
 *
 *  Created on: 30-Sep-2023
 *      Author: aniru
 */

#include "pwm_driver.h"

// LEDCPWM
#if PWM_TYPE == 1
ledc_timer_config_t ledc_timer[LEDC_CH_NUM] = { { .duty_resolution = LEDC_TIMER_10_BIT,  // resolution of PWM duty
		.freq_hz = PWM_FREQ,                   // frequency of PWM signal
		.speed_mode = LEDC_HS_MODE,            // timer mode
		.timer_num = LEDC_HS_TIMER0,           // timer index
		.clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
		}, { .duty_resolution = LEDC_TIMER_10_BIT,  // resolution of PWM duty
				.freq_hz = PWM_FREQ,                   // frequency of PWM signal
				.speed_mode = LEDC_HS_MODE,            // timer mode
				.timer_num = LEDC_HS_TIMER1,           // timer index
				.clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
		}, { .duty_resolution = LEDC_TIMER_10_BIT,  // resolution of PWM duty
				.freq_hz = PWM_FREQ,                   // frequency of PWM signal
				.speed_mode = LEDC_HS_MODE,            // timer mode
				.timer_num = LEDC_HS_TIMER2,           // timer index
				.clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
		}, { .duty_resolution = LEDC_TIMER_10_BIT,  // resolution of PWM duty
				.freq_hz = PWM_FREQ,                   // frequency of PWM signal
				.speed_mode = LEDC_HS_MODE,            // timer mode
				.timer_num = LEDC_HS_TIMER3,           // timer index
				.clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
		}, };

ledc_channel_config_t ledc_channel[LEDC_CH_NUM] = { { .channel = LEDC_HS_CH0_CHANNEL, .duty = 0, .gpio_num = LEDC_HS_CH0_GPIO, .speed_mode = LEDC_HS_MODE, .hpoint = 0, .timer_sel = LEDC_HS_TIMER0,
		.flags.output_invert = 0 }, { .channel = LEDC_HS_CH1_CHANNEL, .duty = 0, .gpio_num = LEDC_HS_CH1_GPIO, .speed_mode = LEDC_HS_MODE, .hpoint = 0, .timer_sel = LEDC_HS_TIMER1,
		.flags.output_invert = 0 }, { .channel = LEDC_HS_CH2_CHANNEL, .duty = 0, .gpio_num = LEDC_HS_CH2_GPIO, .speed_mode = LEDC_HS_MODE, .hpoint = 0, .timer_sel = LEDC_HS_TIMER2,
		.flags.output_invert = 0 }, { .channel = LEDC_HS_CH3_CHANNEL, .duty = 0, .gpio_num = LEDC_HS_CH3_GPIO, .speed_mode = LEDC_HS_MODE, .hpoint = 0, .timer_sel = LEDC_HS_TIMER3	,
		.flags.output_invert = 0 }, };
#endif

// MCPWM
#if PWM_TYPE == 2

static enum channel {
	CH1 = 0, CH2, CH3, CH4
};

//timer config and handle for servo modules
typedef struct {
	mcpwm_timer_config_t config;
	mcpwm_timer_handle_t handle;
} mcpwm_timer_obj_t;
mcpwm_timer_obj_t tim1;

typedef struct {
	mcpwm_oper_handle_t operator;
	mcpwm_operator_config_t operator_config;
	mcpwm_gen_handle_t servo1_op;
	mcpwm_gen_handle_t servo2_op;
	mcpwm_generator_config_t servo1_config;
	mcpwm_generator_config_t servo2_config;
	mcpwm_cmpr_handle_t servo1_comparator;
	mcpwm_cmpr_handle_t servo2_comparator;
	mcpwm_comparator_config_t servo1_comparator_config;
	mcpwm_comparator_config_t servo2_comparator_config;
} servo_goup_handle_t;

servo_goup_handle_t CW_servo;              //M1 and M3
servo_goup_handle_t CCW_servo;              //M2 and M4
//servo_goup_handle_t AUX_servo;

//Timer initialization
void mcpwm_timer_init() {
	tim1.config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
	tim1.config.group_id = 0;
	tim1.config.resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ;
	tim1.config.period_ticks = SERVO_TIMEBASE_PERIOD;
	tim1.config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
	ESP_ERROR_CHECK(mcpwm_new_timer(&tim1.config, &tim1.handle));

	ESP_ERROR_CHECK(mcpwm_timer_enable(tim1.handle));
	ESP_ERROR_CHECK(mcpwm_timer_start_stop(tim1.handle, MCPWM_TIMER_START_NO_STOP));
}

void CW_servo_init() {
	//set config parameters for CW servo controller
	CW_servo.operator_config.group_id = 0;
	ESP_ERROR_CHECK(mcpwm_new_operator(&CW_servo.operator_config, &CW_servo.operator));
	ESP_ERROR_CHECK(mcpwm_operator_connect_timer(CW_servo.operator, tim1.handle));
	CW_servo.servo1_comparator_config.flags.update_cmp_on_tez = true;
	CW_servo.servo2_comparator_config.flags.update_cmp_on_tez = true;

	//	attach comparator to both CW servo handlers
	ESP_ERROR_CHECK(mcpwm_new_comparator(CW_servo.operator, &CW_servo.servo1_comparator_config, &CW_servo.servo1_comparator));
	ESP_ERROR_CHECK(mcpwm_new_comparator(CW_servo.operator, &CW_servo.servo2_comparator_config, &CW_servo.servo2_comparator));

	//	attach pwm generator module
	CW_servo.servo1_config.gen_gpio_num = SERVO_1_PULSE_GPIO;
	CW_servo.servo2_config.gen_gpio_num = SERVO_3_PULSE_GPIO;
	ESP_ERROR_CHECK(mcpwm_new_generator(CW_servo.operator, &CW_servo.servo1_config, &CW_servo.servo1_op));
	ESP_ERROR_CHECK(mcpwm_new_generator(CW_servo.operator, &CW_servo.servo2_config, &CW_servo.servo2_op));

	//Timer and comparator event action for CW_servo1
	ESP_ERROR_CHECK(
			mcpwm_generator_set_actions_on_timer_event(CW_servo.servo1_op, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH), MCPWM_GEN_TIMER_EVENT_ACTION_END()));
	ESP_ERROR_CHECK(
			mcpwm_generator_set_actions_on_compare_event(CW_servo.servo1_op, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, CW_servo.servo1_comparator, MCPWM_GEN_ACTION_LOW), MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

	//Timer and comparator event action for CW_servo2
	ESP_ERROR_CHECK(
			mcpwm_generator_set_actions_on_timer_event(CW_servo.servo2_op, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH), MCPWM_GEN_TIMER_EVENT_ACTION_END()));
	ESP_ERROR_CHECK(
			mcpwm_generator_set_actions_on_compare_event(CW_servo.servo2_op, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, CW_servo.servo2_comparator, MCPWM_GEN_ACTION_LOW), MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

	//intialize comparator value to zero
	ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(CW_servo.servo1_comparator, 0));
	ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(CW_servo.servo2_comparator, 0));
}

void CCW_servo_init() {
	//set config parameters for CW servo controller
	CCW_servo.operator_config.group_id = 0;
	ESP_ERROR_CHECK(mcpwm_new_operator(&CCW_servo.operator_config, &CCW_servo.operator));
	ESP_ERROR_CHECK(mcpwm_operator_connect_timer(CCW_servo.operator, tim1.handle));
	CCW_servo.servo1_comparator_config.flags.update_cmp_on_tez = true;
	CCW_servo.servo2_comparator_config.flags.update_cmp_on_tez = true;

	//	attach comparator to both CW servo handlers
	ESP_ERROR_CHECK(mcpwm_new_comparator(CCW_servo.operator, &CCW_servo.servo1_comparator_config, &CCW_servo.servo1_comparator));
	ESP_ERROR_CHECK(mcpwm_new_comparator(CCW_servo.operator, &CCW_servo.servo2_comparator_config, &CCW_servo.servo2_comparator));

	//	attach pwm generator module
	CCW_servo.servo1_config.gen_gpio_num = SERVO_2_PULSE_GPIO;
	CCW_servo.servo2_config.gen_gpio_num = SERVO_4_PULSE_GPIO;
	ESP_ERROR_CHECK(mcpwm_new_generator(CCW_servo.operator, &CCW_servo.servo1_config, &CCW_servo.servo1_op));
	ESP_ERROR_CHECK(mcpwm_new_generator(CCW_servo.operator, &CCW_servo.servo2_config, &CCW_servo.servo2_op));

	//Timer and comparator event action for CW_servo1
	ESP_ERROR_CHECK(
			mcpwm_generator_set_actions_on_timer_event(CCW_servo.servo1_op, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH), MCPWM_GEN_TIMER_EVENT_ACTION_END()));
	ESP_ERROR_CHECK(
			mcpwm_generator_set_actions_on_compare_event(CCW_servo.servo1_op, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, CCW_servo.servo1_comparator, MCPWM_GEN_ACTION_LOW), MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

	//Timer and comparator event action for CW_servo2
	ESP_ERROR_CHECK(
			mcpwm_generator_set_actions_on_timer_event(CCW_servo.servo2_op, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH), MCPWM_GEN_TIMER_EVENT_ACTION_END()));
	ESP_ERROR_CHECK(
			mcpwm_generator_set_actions_on_compare_event(CCW_servo.servo2_op, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, CCW_servo.servo2_comparator, MCPWM_GEN_ACTION_LOW), MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

	//intialize comparator value to zero
	ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(CCW_servo.servo1_comparator, 0));
	ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(CCW_servo.servo2_comparator, 0));
}

static inline uint32_t angle_to_pwm(float angle, float angle_ll, float angle_ul, float PWM_LL, float PWM_UL) {
	if (angle <= angle_ll)
		return PWM_LL;
	else if (angle > angle_ul)
		return PWM_UL;

	uint32_t pwm = (((angle - angle_ll) * (PWM_UL - PWM_LL)) / (angle_ul - angle_ll) + PWM_LL);
	if (pwm >= PWM_UL) {
		return PWM_UL;
	} else if (pwm <= PWM_LL) {
		return PWM_LL;
	} else {
		return pwm;
	}
}

#endif

// DSHOT
#if PWM_TYPE == 3
#endif
void pwm_init() {

#if PWM_TYPE == 1
	for (int i = 0; i < LEDC_CH_NUM; i++) {
		ledc_timer_config(&ledc_timer[i]);
	}

	for (int i = 0; i < LEDC_CH_NUM; i++) {
		ledc_channel_config(&ledc_channel[i]);
	}
#endif

#if PWM_TYPE == 2
	mcpwm_timer_init();              //same timer for all channels
	CW_servo_init();
	CCW_servo_init();
#endif

#if PWM_TYPE == 3
#endif
}

void pwm_write(uint16_t channel, float value) {
#if PWM_TYPE == 1
	static float pwm = 0;
	pwm = ((value / 100) * 1024);
//	printf("pwm = %f\n",pwm);
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_HS_MODE, ledc_channel[channel].channel, pwm));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_HS_MODE, ledc_channel[channel].channel));
#endif

#if PWM_TYPE == 2
	switch (channel) {
	case CH1:
		ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(CW_servo.servo1_comparator, angle_to_pwm(value, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE, SERVO_MIN_PULSEWIDTH_US, SERVO_MAX_PULSEWIDTH_US)));
		break;
	case CH2:
		ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(CCW_servo.servo1_comparator, angle_to_pwm(value, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE, SERVO_MIN_PULSEWIDTH_US, SERVO_MAX_PULSEWIDTH_US)));
		break;
	case CH3:
		ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(CW_servo.servo2_comparator, angle_to_pwm(value, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE, SERVO_MIN_PULSEWIDTH_US, SERVO_MAX_PULSEWIDTH_US)));
		break;
	case CH4:
		ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(CCW_servo.servo2_comparator, angle_to_pwm(value, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE, SERVO_MIN_PULSEWIDTH_US, SERVO_MAX_PULSEWIDTH_US)));
		break;
	}
#endif
}
