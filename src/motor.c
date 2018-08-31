#include <stdlib.h>

#include "stm32f1xx_hal.h"
#include "motor.h"
#include "config.h"
#include "constants.h"
#include "delay.h"
#include "uart.h"

extern IWDG_HandleTypeDef hiwdg;
extern volatile struct UART uart;

struct Motor motor_L;
struct Motor motor_R;

extern volatile int8_t status;
extern volatile uint32_t last_rx_time;

static const uint8_t HALL_LOOKUP[6] = {
		1, 5, 0, 3, 2, 4
};

static const int8_t REVERSE_HALL_LOOKUP[6][4] = {
		{ 0, 1, 2, -1 },
		{ 0, 2, 1,  1 },
		{ 2, 0, 1, -1 },
		{ 2, 1, 0,  1 },
		{ 1, 2, 0, -1 },
		{ 1, 0, 2,  1 }
};

static const uint32_t TIM_CHANNELS[3] = {
		TIM_CHANNEL_1,
		TIM_CHANNEL_2,
		TIM_CHANNEL_3
};

#if CONTROL_METHOD == SINUSOIDAL_CONTROL
static const float DUTY_SINUSOIDAL[96]= {
	0.86328125, 0.89453125, 0.921875, 0.9453125, 0.96484375, 0.9765625, 0.98828125, 0.99609375,
	0.99609375, 0.99609375, 0.98828125, 0.98046875, 0.96484375, 0.9453125, 0.92578125, 0.8984375,
	0.8671875, 0.890625, 0.91796875, 0.94140625, 0.9609375, 0.9765625, 0.98828125, 0.99609375,
	0.99609375, 0.99609375, 0.9921875, 0.98046875, 0.96484375, 0.94921875, 0.92578125, 0.8984375,
	0.87109375, 0.8359375, 0.796875, 0.7578125, 0.71484375, 0.66796875, 0.6171875, 0.5625,
	0.5078125, 0.453125, 0.39453125, 0.33203125, 0.26953125, 0.20703125, 0.140625, 0.078125,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	0.0078125, 0.07421875, 0.13671875, 0.203125, 0.265625, 0.328125, 0.38671875, 0.44921875,
	0.49609375, 0.5546875, 0.60546875, 0.65625, 0.703125, 0.75, 0.7890625, 0.828125
};
#endif

static float BASE_DUTY_LOOKUP[NUM_PHASES][DUTY_STEPS];

// ----------------------PRIVATE----------------------
// motor control
static void motor_init(struct Motor *motor);
static void motor_start(struct Motor *motor);
static void motor_speed(struct Motor *motor, int16_t rpm);
static void motor_calibrate(struct Motor *motor, int8_t calibration_dir, uint8_t power);
static void motor_pwm(struct Motor *motor, float value_percent);
static void motor_stop(struct Motor *motor);

// interrupts
static void motor_HallSensor_init(struct Motor *motor);
static void motor_TIM_PWM_init(struct Motor *motor);
static void motor_TIM_Duty_init(struct Motor *motor);
static void motor_TIM_Speed_init(struct Motor *motor);

// mosfet control
static void motor_low_on(struct Motor *motor, uint8_t channel);
static void motor_low_off(struct Motor *motor, uint8_t channel);
static void motor_high_on(struct Motor *motor, uint8_t channel);
static void motor_high_off(struct Motor *motor, uint8_t channel);
static void motor_set_pwm(struct Motor *motor, uint8_t channel, float value);
static void motor_set_pwm_all(struct Motor *motor, float value);

// helper methods
static int motor_get_position(struct Motor *motor);
static int in_range(int x);

// ----------------------PUBLIC----------------------
/* Configure both motors and set them up.
 * timer instances + GPIO pins for each motor
 */
void motors_setup_and_init() {
	//motor L config
	motor_L.setup.side = 'L';

	motor_L.setup.htim_pwm.Instance = TIM8;
	motor_L.setup.TIM_PWM_IRQn = TIM8_CC_IRQn;
	motor_L.setup.htim_duty.Instance = TIM6;
	motor_L.setup.TIM_DUTY_IRQn = TIM6_IRQn;
	motor_L.setup.htim_speed.Instance = TIM4;
	motor_L.setup.TIM_SPEED_IRQn = TIM4_IRQn;

	motor_L.setup.HALL_PORT = GPIOB;
	motor_L.setup.HALL_PINS[0] = GPIO_PIN_5;
	motor_L.setup.HALL_PINS[1] = GPIO_PIN_6;
	motor_L.setup.HALL_PINS[2] = GPIO_PIN_7;
	motor_L.setup.EXTI_IRQn = EXTI9_5_IRQn;

	motor_L.setup.OFFSET_POS_HALL = L_POS_OFFSET;
	motor_L.setup.OFFSET_NEG_HALL = L_NEG_OFFSET;
	motor_L.setup.OFFSET_DIR = L_WHEEL_DIR;

	motor_L.setup.GPIO_LOW_PORTS[0] = GPIOA;
	motor_L.setup.GPIO_LOW_CH_PINS[0] = GPIO_PIN_7;
	motor_L.setup.GPIO_LOW_PORTS[1] = GPIOB;
	motor_L.setup.GPIO_LOW_CH_PINS[1] = GPIO_PIN_0;
	motor_L.setup.GPIO_LOW_PORTS[2] = GPIOB;
	motor_L.setup.GPIO_LOW_CH_PINS[2] = GPIO_PIN_1;

	motor_L.setup.GPIO_HIGH_PORT = GPIOC;
	motor_L.setup.GPIO_HIGH_CH_PINS = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;


	//motor R config
	motor_R.setup.side = 'R';

	motor_R.setup.htim_pwm.Instance = TIM1;
	motor_R.setup.TIM_PWM_IRQn = TIM1_CC_IRQn;
	motor_R.setup.htim_duty.Instance = TIM7;
	motor_R.setup.TIM_DUTY_IRQn = TIM7_IRQn;
	motor_R.setup.htim_speed.Instance = TIM3;
	motor_R.setup.TIM_SPEED_IRQn = TIM3_IRQn;

	motor_R.setup.HALL_PORT = GPIOC;
	motor_R.setup.HALL_PINS[0] = GPIO_PIN_10;
	motor_R.setup.HALL_PINS[1] = GPIO_PIN_11;
	motor_R.setup.HALL_PINS[2] = GPIO_PIN_12;
	motor_R.setup.EXTI_IRQn = EXTI15_10_IRQn;

	motor_R.setup.OFFSET_POS_HALL = R_POS_OFFSET;
	motor_R.setup.OFFSET_NEG_HALL = R_NEG_OFFSET;
	motor_R.setup.OFFSET_DIR = R_WHEEL_DIR;

	motor_R.setup.GPIO_LOW_PORTS[0] = GPIOB;
	motor_R.setup.GPIO_LOW_CH_PINS[0] = GPIO_PIN_13;
	motor_R.setup.GPIO_LOW_PORTS[1] = GPIOB;
	motor_R.setup.GPIO_LOW_CH_PINS[1] = GPIO_PIN_14;
	motor_R.setup.GPIO_LOW_PORTS[2] = GPIOB;
	motor_R.setup.GPIO_LOW_CH_PINS[2] = GPIO_PIN_15;

	motor_R.setup.GPIO_HIGH_PORT = GPIOA;
	motor_R.setup.GPIO_HIGH_CH_PINS = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;

	motor_init(&motor_L);
	motor_init(&motor_R);

	//SETUP DUTY LOOKUP
	int i;
	double y = motor_L.uwPeriodValue * 0.01;
	for (i = 0; i < DUTY_STEPS; i++) {

#if CONTROL_METHOD == SINUSOIDAL_CONTROL
		//sinusoidal
		int j;
		for (j = 0; j < NUM_PHASES; j++) {
			BASE_DUTY_LOOKUP[j][i] = y * DUTY_SINUSOIDAL[DUTY_STEPS * j + i];
		}
#elif CONTROL_METHOD == TRAPEZOIDAL_CONTROL
		// trapezoidal
		// index 0 and 1 should be high
		BASE_DUTY_LOOKUP[0][i] = y;
		BASE_DUTY_LOOKUP[1][i] = y;

		// descending
		BASE_DUTY_LOOKUP[2][i] = 0;

		// index 3 and 4 should be low
		BASE_DUTY_LOOKUP[3][i] = 0;
		BASE_DUTY_LOOKUP[4][i] = 0;

		//ascending
		BASE_DUTY_LOOKUP[5][i] = 0;
#endif
  }
}

/* Stop both motors.
 */
void motors_stop() {
	motor_stop(&motor_L);
	motor_stop(&motor_R);
}

/* Calibrate the motors by doing both directions for both the wheels.
 */
void motors_calibrate() {
	HAL_NVIC_DisableIRQ(motor_L.setup.TIM_DUTY_IRQn);
	HAL_NVIC_DisableIRQ(motor_L.setup.TIM_SPEED_IRQn);

	HAL_NVIC_DisableIRQ(motor_R.setup.TIM_DUTY_IRQn);
	HAL_NVIC_DisableIRQ(motor_R.setup.TIM_SPEED_IRQn);

	motor_calibrate(&motor_L, 1, 0);
	motor_calibrate(&motor_L, -1, 0);

	motor_calibrate(&motor_R, 1, 0);
	motor_calibrate(&motor_R, -1, 0);
}

/* Set the speed for both motors.
 */
void motors_speeds(int16_t l_rpm, int16_t r_rpm) {
	motor_speed(&motor_L, l_rpm);
	motor_speed(&motor_R, r_rpm);
}

//-------------------------------interrupt callbacks-------------------------------
/* This is the interrupt function for whenever the hall sensor readings change.
 */
void HALL_ISR_Callback(struct Motor *motor) {

}

/* This is the interrupt function to change the duty cycle 16x per commutation phase.
 * Duration: ~15 microseconds
 */
void Duty_ISR_Callback(struct Motor *motor) {
	int i, index;

	if (motor->stop) {
		return;
	}

	if (motor->timer_duty_cnt == DUTY_STEPS) {
		return;
	}

	if (motor->direction > 0) {
		index = motor->timer_duty_cnt;
	} else {
		index = DUTY_STEPS - 1 - motor->timer_duty_cnt;
	}

	for (i = 0; i < 3; i++) {
		motor->PWM_DUTIES[i] = ((motor->DUTY_LOOKUP[in_range(2*i + motor->next_position)][index]));
		motor_set_pwm(motor, i, motor->PWM_DUTIES[i]);
	}

	motor->timer_duty_cnt += 1;
}

/* This is the interrupt function to change the commutation phase.
 * It also checks to make sure the heart beat is valid.
 * Duration: ~25 microseconds
 */
void Speed_ISR_Callback(struct Motor *motor) {
	// if no new data in a second, stop!!
	if (HAL_GetTick() - last_rx_time > HEARTBEAT_PERIOD) {
		motor_stop(motor);
		SET_ERROR_BIT(status, STATUS_HEARTBEAT_MISSING);
	} else {
		CLR_ERROR_BIT(status, STATUS_HEARTBEAT_MISSING);
	}

	if (motor->stop) {
		return;
	}

	static int newPos;
	if (motor->direction > 0) {
		newPos = in_range(motor_get_position(motor) + motor->setup.OFFSET_POS_HALL);
	} else {
		newPos = in_range(motor_get_position(motor) + motor->setup.OFFSET_NEG_HALL);
	}

	if (newPos == motor->position) {
		motor_pwm(motor, motor->pwm + motor->pos_increment);
	} else if (in_range(newPos - motor->position - motor->direction) != 0) {
		motor_pwm(motor, motor->pwm - motor->neg_increment);
	}

	motor->position = newPos;

#if CONTROL_METHOD == TRAPEZOIDAL_CONTROL
	// turn off the last things
	motor_high_off(motor, REVERSE_HALL_LOOKUP[motor->next_position][0]);
	motor_low_off(motor, REVERSE_HALL_LOOKUP[motor->next_position][2]);
#endif

	// new position
	motor->next_position = in_range(motor->position + motor->direction);

	//turn off the wrong things:
	motor_low_off(motor, REVERSE_HALL_LOOKUP[motor->next_position][0]);
	motor_low_off(motor, REVERSE_HALL_LOOKUP[motor->next_position][1]);
	motor_high_off(motor, REVERSE_HALL_LOOKUP[motor->next_position][2]);

	//set the pwm duties
	motor->timer_duty_cnt = 0;
	Duty_ISR_Callback(motor);

	//based on next position, figure out what the next channels to turn on are
	motor_set_pwm(motor, REVERSE_HALL_LOOKUP[motor->next_position][0], motor->PWM_DUTIES[REVERSE_HALL_LOOKUP[motor->next_position][0]]);
	motor_high_on(motor, REVERSE_HALL_LOOKUP[motor->next_position][0]);

	motor_set_pwm(motor, REVERSE_HALL_LOOKUP[motor->next_position][1], motor->PWM_DUTIES[REVERSE_HALL_LOOKUP[motor->next_position][1]]);
#if CONTROL_METHOD == SINUSOIDAL_CONTROL
	motor_high_on(motor, REVERSE_HALL_LOOKUP[motor->next_position][1]); //not necessary for trapezoidal
#endif

	motor_low_on(motor, REVERSE_HALL_LOOKUP[motor->next_position][2]);

	__HAL_TIM_SET_AUTORELOAD(&(motor->setup.htim_duty), (motor->speed >> 4) - 1);
}

// ----------------------PRIVATE----------------------
// motor control

/* Initialize all the variables and all the timers related to a motor.
 */
static void motor_init(struct Motor *motor) {
	motor_speed(motor, MIN_SPEED);

	motor->position = motor_get_position(motor);
	motor->next_position = motor->position;

	motor->pwm = 0;

	motor->pos_increment = 0;
	motor->neg_increment = 0;

	motor_TIM_PWM_init(motor);
	motor_TIM_Duty_init(motor);
	motor_TIM_Speed_init(motor);
	motor_HallSensor_init(motor);
	motor_stop(motor);

	motor->direction = 1;
	motor->stop = 1;

	// pwm lines: +1, const, or -1
	motor->PWM_DUTIES[0] = 0;
	motor->PWM_DUTIES[1] = 0;
	motor->PWM_DUTIES[2] = 0;

	motor->timer_duty_cnt = 0;
}

/* Start the motor by enabling the interrupts and
 * setting the duty cycles to 0.
 */
static void motor_start(struct Motor *motor) {
	motor_set_pwm_all(motor, 0);
	HAL_NVIC_SetPriority(motor->setup.EXTI_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(motor->setup.EXTI_IRQn);
	motor_pwm(motor, 0);
	motor->stop = 0;
}

/* Set the speed for a motor by setting the timer to the right value.
 * If the speed is out of range, one of the error bits is set.
 */
static void motor_speed(struct Motor *motor, int16_t rpm) {
	if (rpm == 0) {
		motor_stop(motor);
		return;
	}

	//check it is in the valid range
	int absrpm = rpm;
	if (absrpm < 0) {
		absrpm = 0 - absrpm;
	}

	if (absrpm > MAX_SPEED) {
		absrpm = MAX_SPEED;
		SET_ERROR_BIT(status, STATUS_SPEED_OUT_OF_BOUNDS);
	} else if (absrpm < MIN_SPEED) { //min
		motor_stop(motor);
		SET_ERROR_BIT(status, STATUS_SPEED_OUT_OF_BOUNDS);
		return;
	} else {
		CLR_ERROR_BIT(status, STATUS_SPEED_OUT_OF_BOUNDS);
	}

	//1000000 microseconds / second * 60 seconds / minute
	float tick_speed = 1000000 * 60 / (WHEEL_HALL_COUNTS * rpm);
	int16_t old_speed = motor->speed;

	if (tick_speed < 0) {
		motor->direction = -1 * motor->setup.OFFSET_DIR;
		motor->speed = -1 * (int16_t) tick_speed; //absolute value of <speed>
	} else {
		motor->direction = motor->setup.OFFSET_DIR;
		motor->speed = (int16_t) tick_speed;
	}

	if (old_speed != motor->speed) {
		if (old_speed < motor->speed) {
			motor->pos_increment = 1;
			motor->neg_increment = 0.1;
		} else if (old_speed > motor->speed) {
			motor->pos_increment = 0.1;
			motor->neg_increment = 2;
		}

		// set the register of the next thing
		__HAL_TIM_SET_AUTORELOAD(&(motor->setup.htim_speed), motor->speed - 1);
	}

	if (motor->stop == 1) {
		motor_start(motor);
	}

}

/* Calibrate a wheel by slowly increasing the pwm duty cycle until it moves just enough.
 */
static void motor_calibrate(struct Motor *motor, int8_t calibration_dir, uint8_t power) {

	HAL_IWDG_Refresh(&hiwdg);   //819mS

	uint8_t calibrate_positions[NUM_PHASES];
	int i, j, offset_dir;
	int delay = 200;

	motor_low_off(motor, 0);
	motor_low_off(motor, 1);
	motor_low_off(motor, 2);
	motor_high_off(motor, 0);
	motor_high_off(motor, 1);
	motor_high_off(motor, 2);

	// oh god something is broken why is the power ramping up so high, emergency exit
	if (power > 100) {
		motor_set_pwm_all(motor, 0);
		while (!Uart_is_TX_free());
		sprintf((char *) &uart.TX_buffer[0], "max limited power reached, probably something is wrong\n");
		Uart_TX((char *) &uart.TX_buffer[0]);
		return;
	}

	motor_set_pwm_all(motor, power);

	//let it warm up through the first cycle
	for (i = 0; i < 2; i++) {

		// go through all the phases
		for (j = 0; j < NUM_PHASES; j++) {
			HAL_IWDG_Refresh(&hiwdg);   //819mS

			motor_high_off(motor, REVERSE_HALL_LOOKUP[in_range(calibration_dir * (j - 1))][0]);
			motor_low_off(motor, REVERSE_HALL_LOOKUP[in_range(calibration_dir * (j - 1))][2]);

			motor_high_on(motor, REVERSE_HALL_LOOKUP[in_range(calibration_dir * j)][0]);
			motor_low_on(motor, REVERSE_HALL_LOOKUP[in_range(calibration_dir * j)][2]);

			delay_ms(delay);
			calibrate_positions[j] = motor_get_position(motor);
		}

		offset_dir = calibrate_positions[0] - calibrate_positions[NUM_PHASES - 1];

		// didn't really move, ramp up power
		if (offset_dir == 0) {
			motor_calibrate(motor, calibration_dir, power + 10);
			return;
		}
	}

	motor_low_off(motor, 0);
	motor_low_off(motor, 1);
	motor_low_off(motor, 2);
	motor_high_off(motor, 0);
	motor_high_off(motor, 1);
	motor_high_off(motor, 2);
	motor_set_pwm_all(motor, 0);

	if (offset_dir > NUM_PHASES / 2) {
		offset_dir -= NUM_PHASES;
	} else if (offset_dir < -NUM_PHASES / 2) {
		offset_dir += NUM_PHASES;
	}

	// offset direction should be 1 or -1
	if (offset_dir != 1 && offset_dir != -1) {
		motor_calibrate(motor, calibration_dir, power + 1);
		return;
	}

	// make sure all the offsets are right
	for (i = 0; i < NUM_PHASES - 1; i++) {
		if (in_range(calibrate_positions[i + 1] - calibrate_positions[i]) != in_range(offset_dir)) {
			motor_calibrate(motor, calibration_dir, power + 1);
			return;
		}
	}

	while (!Uart_is_TX_free());
	sprintf((char *) &uart.TX_buffer[0], "%c%+d: %d\n", motor->setup.side, offset_dir, in_range(-calibrate_positions[NUM_PHASES - 1]));
	Uart_TX((char *) &uart.TX_buffer[0]);
}

/* Set the pwm for a motor - calculate what the duty cycle values should
 * be at each interval every time the pwm changes.
 *
 * Error bit will be set if the pwm value is over the limit.
 */
static void motor_pwm(struct Motor *motor, float value_percent) {
	int i, j;

	if (value_percent > MAX_POWER_PERCENT) {
		SET_ERROR_BIT(status, STATUS_MAX_POWER_REACHED);
		value_percent = MAX_POWER_PERCENT;
	} else if (value_percent < 0) {
		value_percent = 0;
	} else {
		CLR_ERROR_BIT(status, STATUS_MAX_POWER_REACHED);
	}

	motor->pwm = value_percent;
	HAL_IWDG_Refresh(&hiwdg);   //819mS

	for (i = 0; i < NUM_PHASES; i++) {
		for (j = 0; j < DUTY_STEPS; j++) {
			motor->DUTY_LOOKUP[i][j] = (uint32_t)(BASE_DUTY_LOOKUP[i][j] * motor->pwm);
		}
	}
}

/* Stop everything: turn off all the fets, and set the PWM duty cycles to 0.
 */
static void motor_stop(struct Motor *motor) {
	motor_set_pwm_all(motor, 0);
	motor->stop = 1;

	motor_low_off(motor, 0);
	motor_low_off(motor, 1);
	motor_low_off(motor, 2);
	motor_high_off(motor, 0);
	motor_high_off(motor, 1);
	motor_high_off(motor, 2);

	HAL_NVIC_DisableIRQ(motor->setup.EXTI_IRQn);
	__HAL_GPIO_EXTI_CLEAR_IT(motor->setup.HALL_PINS[0]);
	__HAL_GPIO_EXTI_CLEAR_IT(motor->setup.HALL_PINS[1]);
	__HAL_GPIO_EXTI_CLEAR_IT(motor->setup.HALL_PINS[2]);
}

// ----------------------PRIVATE----------------------
// interrupt functions

/* Set up the GPIO pins for the hall sensor pins.
 * Enable the interrupts for the hall sensors.
 */
static void motor_HallSensor_init(struct Motor *motor) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	if (motor->setup.HALL_PORT == GPIOB)
		__HAL_RCC_GPIOB_CLK_ENABLE();
	else if (motor->setup.HALL_PORT == GPIOC)
		__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pins : HALL_LEFT_A_PIN HALL_LEFT_B_PIN HALL_LEFT_C_PIN */
	GPIO_InitStruct.Pin = motor->setup.HALL_PINS[0]|motor->setup.HALL_PINS[1]|motor->setup.HALL_PINS[2];
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(motor->setup.HALL_PORT, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(motor->setup.EXTI_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(motor->setup.EXTI_IRQn);
}

/* Initialize the timer responsible for the pwm - no interrupts.
 */
static void motor_TIM_PWM_init(struct Motor *motor) {
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	if (motor->setup.htim_pwm.Instance == TIM8)
		__HAL_RCC_TIM8_CLK_ENABLE();
	else if (motor->setup.htim_pwm.Instance == TIM1)
		__HAL_RCC_TIM1_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	motor_low_off(motor, 0);
	motor_low_off(motor, 1);
	motor_low_off(motor, 2);
	motor_high_off(motor, 0);
	motor_high_off(motor, 1);
	motor_high_off(motor, 2);

	motor->uwPeriodValue = (uint32_t) ((SystemCoreClock / PWM_MOTOR) - 1);

	//PULLUPS
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin = motor->setup.GPIO_LOW_CH_PINS[0];
	HAL_GPIO_Init(motor->setup.GPIO_LOW_PORTS[0], &GPIO_InitStruct);
	GPIO_InitStruct.Pin = motor->setup.GPIO_LOW_CH_PINS[1];
	HAL_GPIO_Init(motor->setup.GPIO_LOW_PORTS[1], &GPIO_InitStruct);
	GPIO_InitStruct.Pin = motor->setup.GPIO_LOW_CH_PINS[2];
	HAL_GPIO_Init(motor->setup.GPIO_LOW_PORTS[2], &GPIO_InitStruct);

	//PULLDOWNS
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Pin = motor->setup.GPIO_HIGH_CH_PINS;
	HAL_GPIO_Init(motor->setup.GPIO_HIGH_PORT, &GPIO_InitStruct);

	motor->setup.htim_pwm.Init.Prescaler         = 0;
	motor->setup.htim_pwm.Init.Period            = motor->uwPeriodValue;
	motor->setup.htim_pwm.Init.ClockDivision     = 0;
	motor->setup.htim_pwm.Init.CounterMode       = TIM_COUNTERMODE_UP;
	motor->setup.htim_pwm.Init.RepetitionCounter = 0;
	HAL_TIM_PWM_Init(&(motor->setup.htim_pwm));

	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE; //TIM_AUTOMATICOUTPUT_ENABLE; //
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW; // TIM_BREAKPOLARITY_HIGH; //
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE; //TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE; //TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.DeadTime = 40;   //10;
	HAL_TIMEx_ConfigBreakDeadTime(&(motor->setup.htim_pwm), &sBreakDeadTimeConfig);


	//##-2- Configure the PWM channels #########################################
	// Common configuration for all channels
	sConfigOC.OCMode      = TIM_OCMODE_PWM1; // TIM_OCMODE_PWM2;
	sConfigOC.OCFastMode  = TIM_OCFAST_DISABLE; // TIM_OCFAST_DISABLE;
	sConfigOC.OCPolarity  = TIM_OCPOLARITY_HIGH; //TIM_OCPOLARITY_LOW;//TIM_OCPOLARITY_HIGH; //TIM_OCPOLARITY_LOW;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW; //TIM_OCNPOLARITY_HIGH; //TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET; //TIM_OCIDLESTATE_SET; //TIM_OCNIDLESTATE_RESET; //TIM_OCIDLESTATE_SET;
	sConfigOC.OCNIdleState= TIM_OCNIDLESTATE_RESET; //TIM_OCNIDLESTATE_SET ; //TIM_OCIDLESTATE_SET; //TIM_OCNIDLESTATE_RESET;

	//Set the pulse value
	sConfigOC.Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&(motor->setup.htim_pwm), &sConfigOC, TIM_CHANNEL_1);
	sConfigOC.Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&(motor->setup.htim_pwm), &sConfigOC, TIM_CHANNEL_2);
	sConfigOC.Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&(motor->setup.htim_pwm), &sConfigOC, TIM_CHANNEL_3);

	//Start PWM signals
	HAL_TIM_PWM_Start(&(motor->setup.htim_pwm), TIM_CHANNEL_1);         //CH1
	HAL_TIM_PWM_Start(&(motor->setup.htim_pwm), TIM_CHANNEL_2);         //CH2
	HAL_TIM_PWM_Start(&(motor->setup.htim_pwm), TIM_CHANNEL_3);         //CH3

	motor_set_pwm_all(motor, 0);

	motor_low_off(motor, 0);
	motor_low_off(motor, 1);
	motor_low_off(motor, 2);
	motor_high_off(motor, 0);
	motor_high_off(motor, 1);
	motor_high_off(motor, 2);

	motor->stop = 1;
}

/* Set up the timer to vary the different duty cycles in one commutation phase.
 * This timer has a frequency of 16x the speed timer, so the interrupt happens
 * 16 times per commutation phase.
 */
static void motor_TIM_Duty_init(struct Motor *motor) {
	if (motor->setup.htim_duty.Instance == TIM6)
		__HAL_RCC_TIM6_CLK_ENABLE();
	else if (motor->setup.htim_duty.Instance == TIM7)
		__HAL_RCC_TIM7_CLK_ENABLE();

	// wait until the the next event to update the preload shadow register
	motor->setup.htim_duty.Init.Prescaler = (uint32_t) ((SystemCoreClock / 1000000) - 1);// instructions per microsecond
	motor->setup.htim_duty.Init.Period = (motor->speed >> 4) -1;//period to update the duty cycle, 1/16 of speed
	motor->setup.htim_duty.Init.ClockDivision = 0;
	motor->setup.htim_duty.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_NVIC_SetPriority(motor->setup.TIM_DUTY_IRQn, 2, 0);
	HAL_TIM_Base_Init(&(motor->setup.htim_duty));
	HAL_TIM_Base_Start_IT(&(motor->setup.htim_duty));
	HAL_NVIC_EnableIRQ(motor->setup.TIM_DUTY_IRQn);
}

/* Set up the timer to do the commutation - frequency depends on input rpm.
 */
static void motor_TIM_Speed_init(struct Motor *motor) {
	if (motor->setup.htim_speed.Instance == TIM3)
		__HAL_RCC_TIM3_CLK_ENABLE();
	else if (motor->setup.htim_speed.Instance == TIM4)
		__HAL_RCC_TIM4_CLK_ENABLE();

	motor->setup.htim_speed.Instance->CR1 = motor->setup.htim_speed.Instance->CR1 | TIM_CR1_ARPE_Msk;
	motor->setup.htim_speed.Init.Prescaler = (uint32_t) ((SystemCoreClock / 1000000) - 1); // instructions per microsecond
	motor->setup.htim_speed.Init.Period = motor->speed - 1;
	motor->setup.htim_speed.Init.ClockDivision = 0;
	motor->setup.htim_speed.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_NVIC_SetPriority(motor->setup.TIM_SPEED_IRQn, 1, 0);
	HAL_TIM_Base_Init(&(motor->setup.htim_speed));
	HAL_TIM_Base_Start_IT(&(motor->setup.htim_speed));
	HAL_NVIC_EnableIRQ(motor->setup.TIM_SPEED_IRQn);
}

// ----------------------PRIVATE----------------------
// mosfet functions

/* Turn on the lower mosfet of the half bridge of the corresponding channel.
 */
static void motor_low_on(struct Motor *motor, uint8_t channel) {
	(motor->setup.GPIO_LOW_PORTS[channel])->BSRR = ((uint32_t) motor->setup.GPIO_LOW_CH_PINS[channel]) << 16;
}

/* Turn off the lower mosfet of the half bridge of the corresponding channel.
 */
static void motor_low_off(struct Motor *motor, uint8_t channel) {
	(motor->setup.GPIO_LOW_PORTS[channel])->BSRR = motor->setup.GPIO_LOW_CH_PINS[channel];
}

/* Turn on the upper mosfet of the half bridge of the corresponding channel.
 * (By enabling the comparing for the PWM)/
 */
static void motor_high_on(struct Motor *motor, uint8_t channel) {
	motor->setup.htim_pwm.Instance->CCER = motor->setup.htim_pwm.Instance->CCER | (1 << (channel * 4));  //--> CCXE = 1
}

/* Turn off the upper mosfet of the half bridge of the corresponding channel.
 * (By disabling the comparing for the PWM)
 */
static void motor_high_off(struct Motor *motor, uint8_t channel) {
	motor->setup.htim_pwm.Instance->CCER = motor->setup.htim_pwm.Instance->CCER & ~(1 << (channel * 4));  //--> CCXE = 0
}

/* Set the PWM duty cycle for whatever channel.
 */
static void motor_set_pwm(struct Motor *motor, uint8_t channel, float value) {
	__HAL_TIM_SET_COMPARE(&(motor->setup.htim_pwm), TIM_CHANNELS[channel], value);
}

/* Set the PWM duty cycle for all the channels.
 */
static void motor_set_pwm_all(struct Motor *motor, float value) {
	__HAL_TIM_SET_COMPARE(&(motor->setup.htim_pwm), TIM_CHANNELS[0], value);
	__HAL_TIM_SET_COMPARE(&(motor->setup.htim_pwm), TIM_CHANNELS[1], value);
	__HAL_TIM_SET_COMPARE(&(motor->setup.htim_pwm), TIM_CHANNELS[2], value);
}

// ----------------------PRIVATE----------------------
// helper methods

/* Gets the position deteremind by the hall sensors.
 */
static int motor_get_position(struct Motor *motor) {
	int pos = (motor->setup.HALL_PORT->IDR & (motor->setup.HALL_PINS[0] | motor->setup.HALL_PINS[1] | motor->setup.HALL_PINS[2])) / motor->setup.HALL_PINS[0];
	return HALL_LOOKUP[pos - 1];
}

/* Effectively modulos 6 operator, but without actually doing any division.
 */
static int in_range(int x) {
	const int mod_lookup_table[18] = {
			0, 1, 2, 3, 4, 5,
			0, 1, 2, 3, 4, 5,
			0, 1, 2, 3, 4, 5
	};
	return mod_lookup_table[x + 6];
}
