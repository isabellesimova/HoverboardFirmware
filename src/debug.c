/** This file includes all the peripherals for debugging:
 * buzzer, led, debug pins. Note that each peripheral that
 * is being used needs to be individually initialized.
 */

#include "debug.h"
#include "delay.h"
#include "stm32f1xx_hal.h"

extern IWDG_HandleTypeDef hiwdg;

/************* BUZZER *************/
void buzzer_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_PIN, GPIO_PIN_RESET);
	/*Configure GPIO pin : BUZZER_PIN */
	GPIO_InitStruct.Pin = BUZZER_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStruct);
	buzzer_set(0);
	buzzer_set(1);
	buzzer_set(0);
}

void buzzer_set(uint8_t state) {
	HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_PIN, (GPIO_PinState) state);
}

void buzzer_one_beep(void) {
	for (int i = 0; i < 200; i++) {
		buzzer_set(1);
		delay_us(160);
		buzzer_set(0);
		delay_us(160);
		HAL_IWDG_Refresh(&hiwdg);   //819mS
	}
	buzzer_set(0);
}

void buzzer_two_beeps(void) {
	for (int j = 0; j < 2; j++) {
		for (int i = 0; i < 150; i++) {
			buzzer_set(1);
			delay_us(160);
			buzzer_set(0);
			delay_us(160);
			HAL_IWDG_Refresh(&hiwdg);   //819mS
		}
		buzzer_set(0);
		HAL_Delay(200);
	}
	buzzer_set(0);
}

void buzzer_three_beeps(void) {
	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < 150; i++) {
			buzzer_set(1);
			delay_us(160);
			buzzer_set(0);
			delay_us(160);
			HAL_IWDG_Refresh(&hiwdg);   //819mS
		}
		buzzer_set(0);
		HAL_Delay(200);
	}
	buzzer_set(0);
}

void buzzer_short_beep(void) {
	for (int i = 0; i < 80; i++) {
		buzzer_set(1);
		delay_us(160);
		buzzer_set(0);
		delay_us(160);
		HAL_IWDG_Refresh(&hiwdg);   //819mS
	}
	buzzer_set(0);
}

void buzzer_long_beep(void) {
	for (int i = 0; i < 600; i++) {
		buzzer_set(1);
		delay_us(160);
		buzzer_set(0);
		delay_us(160);
		HAL_IWDG_Refresh(&hiwdg);   //819mS
	}
	buzzer_set(0);
}

/************* LED *************/
void led_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
	/*Configure GPIO pin : LED_PIN */
	GPIO_InitStruct.Pin = LED_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

void led_set(uint8_t state) {
	HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, (GPIO_PinState) state);
}

/************* DEBUG PINS *************/
void debug_pin_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* Debug pin 10 is the blue pin on the side with shorter wires.
 * (opposite the UART connections, opposite the speaker).
 */
void debug_pin_10_ON(void) {
	GPIOB->BSRR = GPIO_PIN_10;
}

void debug_pin_10_OFF(void) {
	GPIOB->BSRR = 0x04000000;
}

/* Debug pin 11 is the green pin on the side with shorter wires.
 * (opposite the UART connections, opposite the speaker).
 */
void debug_pin_11_ON(void) {
	GPIOB->BSRR = GPIO_PIN_11;
}

void debug_pin_11_OFF(void) {
	GPIOB->BSRR = 0x08000000;
}
