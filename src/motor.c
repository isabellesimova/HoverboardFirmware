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

extern uint32_t last_rx_time;
extern volatile int8_t status;

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

static const int DUTY_STEPS_LAG = DUTY_STEPS/3; //a third of duty_steps
static const float DUTY_SINUSOIDAL[DUTY_STEPS]= {
		1.00000, 0.99993, 0.99973, 0.99940, 0.99893, 0.99833, 0.99759, 0.99672, 0.99572, 0.99459, 0.99332, 0.99192, 0.99039, 0.98873, 0.98694, 0.98502, 0.98296, 0.98078, 0.97847, 0.97603, 0.97347, 0.97077, 0.96795, 0.96501, 0.96194, 0.95875, 0.95543, 0.95199, 0.94844, 0.94476, 0.94096, 0.93705, 0.93301, 0.92886, 0.92460, 0.92022, 0.91573, 0.91113, 0.90642, 0.90160, 0.89668, 0.89164, 0.88651, 0.88126, 0.87592, 0.87048, 0.86493, 0.85929, 0.85355, 0.84772, 0.84180, 0.83578, 0.82967, 0.82348, 0.81720, 0.81083, 0.80438, 0.79785, 0.79124, 0.78455, 0.77779, 0.77095, 0.76403, 0.75705, 0.75000, 0.74288, 0.73570, 0.72845, 0.72114, 0.71378, 0.70635, 0.69887, 0.69134, 0.68376, 0.67613, 0.66844, 0.66072, 0.65295, 0.64514, 0.63729, 0.62941, 0.62149, 0.61354, 0.60556, 0.59755, 0.58951, 0.58145, 0.57337, 0.56526, 0.55714, 0.54901, 0.54086, 0.53270, 0.52453, 0.51636, 0.50818, 0.50000, 0.49182, 0.48364, 0.47547, 0.46730, 0.45914, 0.45099, 0.44286, 0.43474, 0.42663, 0.41855, 0.41049, 0.40245, 0.39444, 0.38646, 0.37851, 0.37059, 0.36271, 0.35486, 0.34705, 0.33928, 0.33156, 0.32387, 0.31624, 0.30866, 0.30113, 0.29365, 0.28622, 0.27886, 0.27155, 0.26430, 0.25712, 0.25000, 0.24295, 0.23597, 0.22905, 0.22221, 0.21545, 0.20876, 0.20215, 0.19562, 0.18917, 0.18280, 0.17652, 0.17033, 0.16422, 0.15820, 0.15228, 0.14645, 0.14071, 0.13507, 0.12952, 0.12408, 0.11874, 0.11349, 0.10836, 0.10332, 0.09840, 0.09358, 0.08887, 0.08427, 0.07978, 0.07540, 0.07114, 0.06699, 0.06295, 0.05904, 0.05524, 0.05156, 0.04801, 0.04457, 0.04125, 0.03806, 0.03499, 0.03205, 0.02923, 0.02653, 0.02397, 0.02153, 0.01922, 0.01704, 0.01498, 0.01306, 0.01127, 0.00961, 0.00808, 0.00668, 0.00541, 0.00428, 0.00328, 0.00241, 0.00167, 0.00107, 0.00060, 0.00027, 0.00007, 0.00000, 0.00007, 0.00027, 0.00060, 0.00107, 0.00167, 0.00241, 0.00328, 0.00428, 0.00541, 0.00668, 0.00808, 0.00961, 0.01127, 0.01306, 0.01498, 0.01704, 0.01922, 0.02153, 0.02397, 0.02653, 0.02923, 0.03205, 0.03499, 0.03806, 0.04125, 0.04457, 0.04801, 0.05156, 0.05524, 0.05904, 0.06295, 0.06699, 0.07114, 0.07540, 0.07978, 0.08427, 0.08887, 0.09358, 0.09840, 0.10332, 0.10836, 0.11349, 0.11874, 0.12408, 0.12952, 0.13507, 0.14071, 0.14645, 0.15228, 0.15820, 0.16422, 0.17033, 0.17652, 0.18280, 0.18917, 0.19562, 0.20215, 0.20876, 0.21545, 0.22221, 0.22905, 0.23597, 0.24295, 0.25000, 0.25712, 0.26430, 0.27155, 0.27886, 0.28622, 0.29365, 0.30113, 0.30866, 0.31624, 0.32387, 0.33156, 0.33928, 0.34705, 0.35486, 0.36271, 0.37059, 0.37851, 0.38646, 0.39444, 0.40245, 0.41049, 0.41855, 0.42663, 0.43474, 0.44286, 0.45099, 0.45914, 0.46730, 0.47547, 0.48364, 0.49182, 0.50000, 0.50818, 0.51636, 0.52453, 0.53270, 0.54086, 0.54901, 0.55714, 0.56526, 0.57337, 0.58145, 0.58951, 0.59755, 0.60556, 0.61354, 0.62149, 0.62941, 0.63729, 0.64514, 0.65295, 0.66072, 0.66844, 0.67613, 0.68376, 0.69134, 0.69887, 0.70635, 0.71378, 0.72114, 0.72845, 0.73570, 0.74288, 0.75000, 0.75705, 0.76403, 0.77095, 0.77779, 0.78455, 0.79124, 0.79785, 0.80438, 0.81083, 0.81720, 0.82348, 0.82967, 0.83578, 0.84180, 0.84772, 0.85355, 0.85929, 0.86493, 0.87048, 0.87592, 0.88126, 0.88651, 0.89164, 0.89668, 0.90160, 0.90642, 0.91113, 0.91573, 0.92022, 0.92460, 0.92886, 0.93301, 0.93705, 0.94096, 0.94476, 0.94844, 0.95199, 0.95543, 0.95875, 0.96194, 0.96501, 0.96795, 0.97077, 0.97347, 0.97603, 0.97847, 0.98078, 0.98296, 0.98502, 0.98694, 0.98873, 0.99039, 0.99192, 0.99332, 0.99459, 0.99572, 0.99672, 0.99759, 0.99833, 0.99893, 0.99940, 0.99973, 0.99993
};
static float BASE_DUTY_LOOKUP[DUTY_STEPS];


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
static void motor_fets_on(struct Motor *motor);
static void motor_fets_trapezoidal(struct Motor *motor, int high, int low);
static void motor_fets_off(struct Motor *motor);
static void motor_set_pwm(struct Motor *motor, float value0, float value1, float value2);

// helper methods
static int motor_get_position(struct Motor *motor);
static int in_range(int x);
static uint16_t in_range_duty(struct Motor *motor, int channel, int index);

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

	motor_L.other_motor = &motor_R;
	motor_R.other_motor = &motor_L;

	motor_init(&motor_L);
	motor_init(&motor_R);

	for (int i = 0; i < DUTY_STEPS; i++) {
		BASE_DUTY_LOOKUP[i] = motor_L.pwm_percent_period * DUTY_SINUSOIDAL[i];
	}

}

/* Set the speed for both motors.
 */
void motors_speeds(int16_t l_rpm, int16_t r_rpm) {
	motor_speed(&motor_L, l_rpm);
	motor_speed(&motor_R, r_rpm);
}

/* Update the lookup pwm values for both tables
 */
void motors_pwms() {
	motor_pwm(&motor_L, motor_L.new_pwm);
	motor_pwm(&motor_R, motor_R.new_pwm);
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

//-------------------------------interrupt callbacks-------------------------------
/* This is the interrupt function for whenever the hall sensor readings change.
 */
void HALL_ISR_Callback(struct Motor *motor) {
	// stop -- don't do the commutation
	if (motor->state == STOPPED) {
		return;
	}

	// figure out how much it has already moved - expect it to be one, but just in case.
	static int newPos;
	if (motor->direction > 0) {
		newPos = in_range(motor_get_position(motor) + motor->setup.OFFSET_POS_HALL);
	} else {
		newPos = in_range(motor_get_position(motor) + motor->setup.OFFSET_NEG_HALL);
	}

	static int8_t diff;
	diff = in_range((newPos - motor->position) * motor->direction);
	if (diff > 3) {
		diff = diff - 6;
	}

	motor->this_hall_count = motor->this_hall_count + diff;
	motor->position = newPos;
	motor->next_position = in_range(motor->position + motor->direction);

	// if you finish the spline, stop
	// 0 means no limit
	if (motor->hall_limit != 0 && motor->hall_limit <= motor->this_hall_count) {
		motor_stop(motor);
		return;
	}

}

/* This is the interrupt function to change the duty cycle 16x per commutation phase.
 * Duration: ~15 microseconds
 */
void Duty_ISR_Callback(struct Motor *motor) {
	if (motor->state == STOPPED) {
		return;
	}

	if (motor->state == STARTING) {
		if (motor->timer_duty_cnt % 64 == 0) {
			if (motor->total_hall_count >= 6 && motor->position == in_range(3+motor->direction)){
				motor_fets_on(motor);
				motor->state = SETTING_UP;
				motor->DUTY_LOOKUP_POINTER_NEW = motor->DUTY_LOOKUP_1;
				__HAL_TIM_SET_AUTORELOAD(&(motor->setup.htim_duty), motor->speed - 1);
				__HAL_TIM_SET_AUTORELOAD(&(motor->setup.htim_speed), motor->speed - 1);
				motor->setup.htim_duty.Instance->CNT = 0;
				motor->setup.htim_speed.Instance->CNT = 0;
				motor->timer_duty_cnt = 0;
				return;
			}
			motor_fets_trapezoidal(motor, REVERSE_HALL_LOOKUP[motor->position][0], REVERSE_HALL_LOOKUP[motor->position][2]);
		}
	} else {
		motor_set_pwm(motor, in_range_duty(motor, 0, motor->timer_duty_cnt),
				in_range_duty(motor, motor->direction, motor->timer_duty_cnt + DUTY_STEPS + DUTY_STEPS_LAG * motor->direction),
				in_range_duty(motor, -motor->direction, motor->timer_duty_cnt + DUTY_STEPS + DUTY_STEPS_LAG * 2 * motor->direction));
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
		motors_stop();
		SET_ERROR_BIT(status, STATUS_HEARTBEAT_MISSING);
	} else {
		CLR_ERROR_BIT(status, STATUS_HEARTBEAT_MISSING);
	}

	if (motor->state == STOPPED) {
		return;
	}

	motor->delta = motor->this_hall_count - motor->last_hall_count;
	motor->last_hall_count = motor->this_hall_count;
	motor->total_hall_count += motor->delta;


	if (motor->state == STARTING) {
		if (motor->delta == 0) {
			motor->new_pwm = motor->pwm + 5;
		} else if (motor->delta > 6) {
			motor->new_pwm = motor->pwm - 1;
		}
	} else if (motor->state == SETTING_UP || motor->state == READY_TO_TRANSITION) {
		motor->new_pwm = motor->pwm;
	}
	else if (motor->state == TRANSITIONING) {
		if (motor->delta < MOTOR_SPEED_CHECK) {

			motor->state = GOING;
		} else if (motor->delta >= MOTOR_SPEED_CHECK) {
			motor->new_pwm = motor->pwm - 2;
		}
	} else if (motor->state == GOING) {
		if (motor->delta < MOTOR_SPEED_CHECK) {
			motor->new_pwm = motor->pwm + 1;
		} else if (motor->delta > MOTOR_SPEED_CHECK) {
			motor->new_pwm = motor->pwm - 1;
		}
	}

	// double check the position if no change in a while
	if (motor->delta ==0) {
		HALL_ISR_Callback(motor);
	}

	__HAL_TIM_SET_AUTORELOAD(&(motor->setup.htim_duty), motor->speed - 1);
	__HAL_TIM_SET_AUTORELOAD(&(motor->setup.htim_speed), motor->speed - 1);
	motor->timer_duty_cnt = 0;
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
	motor->new_pwm = 0;

	motor->DUTY_LOOKUP_POINTER_OLD = motor->DUTY_LOOKUP_1;
	motor->DUTY_LOOKUP_POINTER_NEW = motor->DUTY_LOOKUP_1;

	motor_TIM_PWM_init(motor);
	motor_TIM_Duty_init(motor);
	motor_TIM_Speed_init(motor);
	motor_HallSensor_init(motor);
	motor_stop(motor);

	motor->direction = 1;
	motor->state = STOPPED;
	motor->timer_duty_cnt = 0;
}

/* Start the motor by enabling the interrupts and
 * setting the duty cycles to 0.
 */
static void motor_start(struct Motor *motor) {
	motor->delta = MOTOR_SPEED_CHECK;
	motor->last_hall_count = 0;
	motor->this_hall_count = 0;
	motor->total_hall_count = 0;

	if (motor->direction > 0) {
		motor->position = in_range(motor_get_position(motor) + motor->setup.OFFSET_POS_HALL);
	} else {
		motor->position = in_range(motor_get_position(motor) + motor->setup.OFFSET_NEG_HALL);
	}

	motor->state = STARTING;
	motor_set_pwm(motor, 0, 0, 0);
	motor_pwm(motor, 20);
	motor->timer_duty_cnt = 0;

	//enable timers
	__HAL_TIM_SET_AUTORELOAD(&(motor->setup.htim_duty), motor->speed - 1);
	__HAL_TIM_SET_AUTORELOAD(&(motor->setup.htim_speed), motor->speed - 1);
	HAL_NVIC_EnableIRQ(motor->setup.EXTI_IRQn);
	HAL_NVIC_EnableIRQ(motor->setup.TIM_SPEED_IRQn);
	HAL_NVIC_EnableIRQ(motor->setup.TIM_DUTY_IRQn);
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

	if (tick_speed < 0) {
		motor->direction = -1 * motor->setup.OFFSET_DIR;
		motor->speed = -1 * (int16_t) tick_speed; //absolute value of <speed>
	} else {
		motor->direction = motor->setup.OFFSET_DIR;
		motor->speed = (int16_t) tick_speed;
	}

	if (motor->state == STOPPED) {
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
		motor_set_pwm(motor, 0, 0, 0);
		while (!Uart_is_TX_free());
		sprintf((char *) &uart.TX_buffer[0], "max limited power reached, probably something is wrong\n");
		Uart_TX((char *) &uart.TX_buffer[0]);
		return;
	}

	motor_set_pwm(motor, power, power, power);

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
	motor_set_pwm(motor, 0, 0, 0);

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
	int i;

	if (value_percent > MAX_POWER_PERCENT) {
		SET_ERROR_BIT(status, STATUS_MAX_POWER_REACHED);
		value_percent = MAX_POWER_PERCENT;
	} else if (value_percent < 0) {
		value_percent = 0;
	} else {
		CLR_ERROR_BIT(status, STATUS_MAX_POWER_REACHED);
	}

	if (motor->pwm != value_percent) {
		motor->pwm = value_percent;
		if (motor->state == STARTING) {
			for (i = 0; i < DUTY_STEPS; i++) {
				motor->DUTY_LOOKUP_1[i] = (uint16_t)(motor->pwm * motor->pwm_percent_period);
				motor->DUTY_LOOKUP_2[i] = (uint16_t)(motor->pwm * BASE_DUTY_LOOKUP[i] );
			}
		} else {
			if (motor->DUTY_LOOKUP_POINTER_NEW == motor->DUTY_LOOKUP_1) {
				for (i = 0; i < DUTY_STEPS; i++) {
					motor->DUTY_LOOKUP_2[i] = (uint16_t)(motor->pwm * BASE_DUTY_LOOKUP[i]);
				}
			}
		}
	}
}

/* Stop everything: turn off all the fets, and set the PWM duty cycles to 0.
 */
static void motor_stop(struct Motor *motor) {
	motor->state = STOPPED;

	motor_fets_off(motor);
	HAL_NVIC_DisableIRQ(motor->setup.EXTI_IRQn);
	HAL_NVIC_DisableIRQ(motor->setup.TIM_DUTY_IRQn);
	HAL_NVIC_DisableIRQ(motor->setup.TIM_SPEED_IRQn);

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

	motor_fets_off(motor);

	motor->uwPeriodValue = (uint32_t) ((SystemCoreClock / PWM_MOTOR) - 1);
	motor->pwm_percent_period = motor->uwPeriodValue * 0.01;

	//LOW HALF
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Pin = motor->setup.GPIO_LOW_CH_PINS[0];
	HAL_GPIO_Init(motor->setup.GPIO_LOW_PORTS[0], &GPIO_InitStruct);
	GPIO_InitStruct.Pin = motor->setup.GPIO_LOW_CH_PINS[1];
	HAL_GPIO_Init(motor->setup.GPIO_LOW_PORTS[1], &GPIO_InitStruct);
	GPIO_InitStruct.Pin = motor->setup.GPIO_LOW_CH_PINS[2];
	HAL_GPIO_Init(motor->setup.GPIO_LOW_PORTS[2], &GPIO_InitStruct);

	//HIGH HALF
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Pin = motor->setup.GPIO_HIGH_CH_PINS;
	HAL_GPIO_Init(motor->setup.GPIO_HIGH_PORT, &GPIO_InitStruct);

	motor->setup.htim_pwm.Init.Prescaler         = 0;
	motor->setup.htim_pwm.Init.Period            = motor->uwPeriodValue;
	motor->setup.htim_pwm.Init.ClockDivision     = 0;
	motor->setup.htim_pwm.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
	motor->setup.htim_pwm.Init.RepetitionCounter = 0;
	HAL_TIM_PWM_Init(&(motor->setup.htim_pwm));

	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE; //TIM_AUTOMATICOUTPUT_ENABLE; //
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW; // TIM_BREAKPOLARITY_HIGH; //
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE; //TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE; //TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.DeadTime = 32;
	HAL_TIMEx_ConfigBreakDeadTime(&(motor->setup.htim_pwm), &sBreakDeadTimeConfig);

	//##-2- Configure the PWM channels #########################################
	// Common configuration for all channels
	sConfigOC.OCMode      = TIM_OCMODE_PWM1;
	sConfigOC.OCFastMode  = TIM_OCFAST_DISABLE;
	sConfigOC.OCPolarity  = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState= TIM_OCNIDLESTATE_RESET;

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

	motor_set_pwm(motor, 0, 0, 0);
	motor_fets_on(motor);

	motor->state = STOPPED;
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
	motor->setup.htim_duty.Init.Prescaler = (uint32_t) (((SystemCoreClock / 1000000) >> 6) - 1); // instructions per microsecond - 1/64 of speed
	motor->setup.htim_duty.Init.Period = motor->speed - 1;
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

	motor->setup.htim_speed.Init.Prescaler = (uint32_t) ((SystemCoreClock / 1000000) * 6 - 1); // instructions per microsecond
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

/* Turn on all the mosfets of the half bridges
 * (By enabling the PWM and complementary PWM)
 */
static void motor_fets_on(struct Motor *motor) {
	//--> CCXE = 1, CCXNE = 1 for X = 1,2,3
	motor->setup.htim_pwm.Instance->CCER = motor->setup.htim_pwm.Instance->CCER | 0b010101010101;
}

/* Turn on all the mosfets of the half bridges
 * (By enabling the PWM and complementary PWM)
 */
static void motor_fets_trapezoidal(struct Motor *motor, int high, int low) {
	//--> CCXE = 1, CCXNE = 1 for X = 1,2,3
	float pwm[3];
	pwm[0] = pwm[1] = pwm[2] = motor->pwm * motor->pwm_percent_period;
	motor_set_pwm(motor, pwm[0], pwm[1], pwm[2]);
	int16_t bitmask = 1 << (high * 4) | (1 << (low * 4 + 2));
	motor->setup.htim_pwm.Instance->CCER = bitmask | (motor->setup.htim_pwm.Instance->CCER & ~(0b010101010101));
}

/* Turn off all the mosfets of the half bridges
 * (By disabling the comparing for the PWM and complementary PWM)
 */

static void motor_fets_off(struct Motor *motor) {
	//--> CCXE = 0, CCXNE = 0 for X = 1,2,3
	motor->setup.htim_pwm.Instance->CCER = motor->setup.htim_pwm.Instance->CCER & ~(0b010101010101);
}

/* Set the PWM duty cycles.
 */
static void motor_set_pwm(struct Motor *motor, float value0, float value1, float value2) {
	__HAL_TIM_SET_COMPARE(&(motor->setup.htim_pwm),TIM_CHANNELS[0], value0);
	__HAL_TIM_SET_COMPARE(&(motor->setup.htim_pwm),TIM_CHANNELS[1], value1);
	__HAL_TIM_SET_COMPARE(&(motor->setup.htim_pwm),TIM_CHANNELS[2], value2);
}

// ----------------------PRIVATE----------------------
// helper methods

/* Gets the position determined by the hall sensors.
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

static uint16_t in_range_duty(struct Motor *motor, int channel, int index) {
	while (index >= DUTY_STEPS) {
		index-= DUTY_STEPS;
	}

	uint16_t value_to_return = 0;
	if (motor->state == SETTING_UP || motor->state == READY_TO_TRANSITION ) {
		if (channel == 0) {
			if (index == 0) {
				if (motor->state == SETTING_UP) {
					motor->DUTY_LOOKUP_POINTER_NEW = motor->DUTY_LOOKUP_2;
					motor->state = READY_TO_TRANSITION;
				} else if (motor->state == READY_TO_TRANSITION) {
					motor->DUTY_LOOKUP_POINTER_NEW = motor->DUTY_LOOKUP_1;
					motor->state = TRANSITIONING;
				}
			}
			value_to_return = motor->DUTY_LOOKUP_POINTER_NEW[index];
		} else if (channel == 1) {
			if (index < DUTY_STEPS/3) {
				value_to_return = motor->DUTY_LOOKUP_POINTER_NEW[index];
			} else {
				value_to_return = motor->DUTY_LOOKUP_POINTER_OLD[index];
			}
			motor->DUTY_LOOKUP_POINTER_OLD[index] = motor->DUTY_LOOKUP_POINTER_NEW[index];
		} else {
			if (index < DUTY_STEPS * 2/3) {
				value_to_return = motor->DUTY_LOOKUP_POINTER_NEW[index];
			} else {
				value_to_return = motor->DUTY_LOOKUP_POINTER_OLD[index];
			}
		}
	} else {
		if (channel == 0) {
			if (index > DUTY_STEPS/2) {
				value_to_return = motor->DUTY_LOOKUP_POINTER_NEW[index];
			} else {
				value_to_return = motor->DUTY_LOOKUP_POINTER_OLD[index];
			}
		} else if (channel == 1) {
			if (index == DUTY_STEPS/2) {
				if (motor->DUTY_LOOKUP_POINTER_NEW == motor->DUTY_LOOKUP_2) {
					motor->DUTY_LOOKUP_POINTER_NEW = motor->DUTY_LOOKUP_1;
				} else if (motor->DUTY_LOOKUP_1[0] != motor->DUTY_LOOKUP_2[0]) {
					motor->DUTY_LOOKUP_POINTER_NEW = motor->DUTY_LOOKUP_2;
				}
			}
			if (index < DUTY_STEPS/3 || index > DUTY_STEPS/2) {
				value_to_return = motor->DUTY_LOOKUP_POINTER_NEW[index];
			} else {
				value_to_return = motor->DUTY_LOOKUP_POINTER_OLD[index];
			}
		} else {
			if (index < DUTY_STEPS * 2/3 && index > DUTY_STEPS/2) {
				value_to_return = motor->DUTY_LOOKUP_POINTER_NEW[index];
			} else {
				value_to_return = motor->DUTY_LOOKUP_POINTER_OLD[index];
			}
			motor->DUTY_LOOKUP_POINTER_OLD[index] = motor->DUTY_LOOKUP_POINTER_NEW[index];
		}
	}
	return value_to_return;
}
