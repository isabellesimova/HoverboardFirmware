/**
 * This class allows for control of a 3 phase BLDC motor.
 * It uses trapezoidal commutation to start up the motor, and once it has moved a bit, it will switch to sinusoidal control.
 * The sinusoidal control runs between 0 and motor->pwm % duty cycle, and each phase switches to a new pwm% duty cycle when the duty cycle hits 0%.
 */

#ifndef __MOTOR__H
#define __MOTOR__H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "config.h"

#define PWM_MOTOR 25000			//PWM frequency in Hertz
#define MIN_SPEED 11			//rotations per minute
#define MAX_SPEED 360
#define NUM_PHASES 6
#define DUTY_STEPS 16
#define PI 3.14159265358

//non volatile stuff, constant things
struct Motor_setup {
	// just L or R to help with debugging
	char side;

	// related to timer running pwm
	TIM_HandleTypeDef htim_pwm;
	int8_t TIM_PWM_IRQn;

	// timer determining the duty cycle
	TIM_HandleTypeDef htim_duty;
	int8_t TIM_DUTY_IRQn;

	// timer determining the speed
	TIM_HandleTypeDef htim_speed;
	int8_t TIM_SPEED_IRQn;
	int8_t TS_bitmask;

	// hall pins
	GPIO_TypeDef* HALL_PORT;
	int16_t HALL_PINS[3];
	int8_t EXTI_IRQn;

	uint8_t OFFSET_POS_HALL;
	uint8_t OFFSET_NEG_HALL;
	int8_t OFFSET_DIR;

	// pull ups, active when BSRR is reset / low
	GPIO_TypeDef* GPIO_LOW_PORTS[3];
	uint16_t GPIO_LOW_CH_PINS[3];

	// pull downs, active when BSRR is set / high
	GPIO_TypeDef* GPIO_HIGH_PORT;
	uint16_t GPIO_HIGH_CH_PINS;
};

// volatile variables that change on interrupts
struct Motor {
	struct Motor_setup setup;
	volatile uint32_t uwPeriodValue;
	volatile uint8_t position; //hall
	volatile uint8_t next_position; //hall

	volatile float pwm;

	volatile uint16_t speed;
	volatile int8_t direction; //+1 or -1
	volatile uint8_t stop; // 0 or 1

	volatile float pos_increment; // 1 or 0.1
	volatile float neg_increment; // 2 or 0.1

	// pwm lines: +1, 0, or -1
	volatile uint32_t PWM_DUTIES[3];
	volatile uint32_t DUTY_LOOKUP[6][DUTY_STEPS];

	volatile int16_t timer_duty_cnt;
};

void motors_setup_and_init(void);
void motors_stop();
void motors_calibrate();
void motors_speeds(int16_t l_rpm, int16_t r_rpm);

void HALL_ISR_Callback(struct Motor *motor);
void Duty_ISR_Callback(struct Motor *motor);
void Speed_ISR_Callback(struct Motor *motor);

extern void error_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */
