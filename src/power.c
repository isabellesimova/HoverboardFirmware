/** This file includes all the peripherals for power:
 *  on/off button, charging.
 */

#include "power.h"
#include "stm32f1xx_hal.h"

/************* Charging *************/
/* Initialize the GPIO pin that reads if the battery is being charged.
 */
void charging_init(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/*Configure GPIO pin : CHARGING_PIN */
	GPIO_InitStruct.Pin = CHARGING_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(CHARGING_GPIO_PORT, &GPIO_InitStruct);
}

uint8_t is_charging(void){
	return 1 - (uint8_t)HAL_GPIO_ReadPin(CHARGING_GPIO_PORT, CHARGING_PIN);
}


/************* Button *************/
/* Initialize the on/off button and set up the interrupt to toggle the pin.
 */
void button_init(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin in: BUTTON_PIN */
	GPIO_InitStruct.Pin = BUTTON_PIN_IN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUTTON_GPIO_PORT, &GPIO_InitStruct);

	/*Configure GPIO pin output Level */
	HAL_GPIO_WritePin(BUTTON_GPIO_PORT, BUTTON_PIN_OUT, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = BUTTON_PIN_OUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BUTTON_GPIO_PORT, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

uint8_t button_pressed(void) {
	return (uint8_t)HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_PIN_IN);
}

void button_toggle(void){
	HAL_GPIO_TogglePin(BUTTON_GPIO_PORT, BUTTON_PIN_OUT);
}
