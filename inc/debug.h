/** This file includes all the peripherals for debugging:
 * buzzer, led, debug pins. Note that each peripheral that
 * is being used needs to be individually initialized.
 */

#ifndef __DEBUG__H
#define __DEBUG__H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f1xx_hal.h"

#define BUZZER_PIN GPIO_PIN_4
#define BUZZER_GPIO_PORT GPIOA
#define LED_PIN GPIO_PIN_2
#define LED_GPIO_PORT GPIOB

void buzzer_init(void);
void buzzer_set(uint8_t stato);
void buzzer_one_beep(void);
void buzzer_two_beeps(void);
void buzzer_three_beeps(void);
void buzzer_short_beep(void);
void buzzer_long_beep(void);

void led_init(void);
void led_set(uint8_t stato);

void debug_pin_init(void);
void debug_pin_10_ON(void);
void debug_pin_10_OFF(void);
void debug_pin_11_ON(void);
void debug_pin_11_OFF(void);

extern void error_handler(void);

#ifdef __cplusplus
}
#endif

#endif
