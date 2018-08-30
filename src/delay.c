/** This file deals with delays. */

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "delay.h"

TIM_HandleTypeDef TimHandle;

// ----------------------PRIVATE----------------------
void _init_us(void);
void _init_ms(void);
void _stop_timer(void);

// ----------------------PUBLIC----------------------
/* Delay for given number of milliseconds.
 */
void delay_ms(uint32_t mSecs) {
	// init and start timer
	_init_ms();

	// dummy loop with 16 bit count wrap around
	uint32_t start = TIM2->CNT;
	while((TIM2->CNT-start) <= mSecs);

	// stop timer
	_stop_timer();
}

/* Delay for given number of microseconds.
 */
void delay_us(uint32_t uSecs) {
	// init and start timer
	_init_us();

	// dummy loop with 16 bit count wrap around
	uint32_t start = TIM2->CNT;
	while((TIM2->CNT-start) <= uSecs);

	// stop timer
	_stop_timer();
}

// ----------------------PRIVATE----------------------
/* Init & start timer for microseconds delays
 */
void _init_us() {
	__HAL_RCC_TIM2_CLK_ENABLE();
	TimHandle.Instance = TIM2;
	TimHandle.Init.Period = UINT16_MAX;
	TimHandle.Init.Prescaler = (SystemCoreClock/DELAY_TIM_FREQUENCY_US)-1;;
	TimHandle.Init.ClockDivision = 0;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_Base_Init(&TimHandle);
	HAL_TIM_Base_Start(&TimHandle);
}

/* Init & start timer for milliseconds delays
 */
void _init_ms() {
	__HAL_RCC_TIM2_CLK_ENABLE();
	TimHandle.Instance = TIM2;
	TimHandle.Init.Period = UINT16_MAX;
	TimHandle.Init.Prescaler = (SystemCoreClock/DELAY_TIM_FREQUENCY_MS)-1;;
	TimHandle.Init.ClockDivision = 0;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_Base_Init(&TimHandle);
	HAL_TIM_Base_Start(&TimHandle);
}

/* Stop the timer by disabling the clock.
 */
void _stop_timer() {
	HAL_TIM_Base_Stop(&TimHandle);
	__HAL_RCC_TIM2_CLK_DISABLE();
}
