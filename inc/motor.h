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

struct Motor_setup{

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

struct Motor {
	struct Motor_setup setup;
	volatile __IO uint32_t uwPeriodValue;
	volatile __IO uint8_t position; //hall
	volatile __IO uint8_t next_position; //hall

	volatile __IO float pwm;

	volatile __IO uint16_t speed;
	volatile __IO int8_t direction; //+1 or -1
	volatile __IO uint8_t stop; // 0 or 1

	volatile __IO float pos_increment; // 1 or 0.1
	volatile __IO float neg_increment; // 2 or 0.1

	// pwm lines: +1, 0, or -1
	volatile __IO uint32_t PWM_DUTIES[3];
	volatile __IO uint32_t DUTY_LOOKUP[6][DUTY_STEPS];

	volatile __IO int16_t timer_duty_cnt;
};

// PUBLIC
void Motors_setup_and_init(void);
void Motors_speeds(int16_t l_rpm, int16_t r_rpm);
void Motors_stop();
#ifdef CALIBRATION
void Motors_calibrate();
void motor_calibrate(struct Motor *motor, int8_t calibration_dir, uint8_t power);
#endif

// PRIVATE
void motor_init(struct Motor *motor);
void motor_start(struct Motor *motor);
void motor_stop(struct Motor *motor);
void motor_speed(struct Motor *motor, int16_t rpm);
void motor_pwm(struct Motor *motor, float value_percent);

void motor_TIM_PWM_init(struct Motor *motor);
void motor_TIM_Duty_init(struct Motor *motor);
void motor_TIM_Speed_init(struct Motor *motor);
void motor_HallSensor_init(struct Motor *motor);

void motor_Low_ON(struct Motor *motor, uint8_t channel);
void motor_Low_OFF(struct Motor *motor, uint8_t channel);
void motor_High_ON(struct Motor *motor, uint8_t channel);
void motor_High_OFF(struct Motor *motor, uint8_t channel);
void motor_Set_PWM(struct Motor *motor, uint8_t channel, float value);
void motor_Set_PWM_ALL(struct Motor *motor, float value);

int motor_Get_Position(struct Motor *motor);

void HALL_ISR_Callback(struct Motor *motor);
void Duty_ISR_Callback(struct Motor *motor);
void Speed_ISR_Callback(struct Motor *motor);

extern void error_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */
