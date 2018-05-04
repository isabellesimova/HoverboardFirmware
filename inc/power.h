/** This file includes all the peripherals for power:
 *  on/off button, charging.
 */

#ifndef __POWER__H
#define __POWER__H

#ifdef __cplusplus
extern "C" {
#endif 

#include "stm32f1xx_hal.h"

#define CHARGING_PIN GPIO_PIN_12
#define CHARGING_GPIO_PORT GPIOA
#define BUTTON_PIN_IN GPIO_PIN_1
#define BUTTON_PIN_OUT GPIO_PIN_5
#define BUTTON_GPIO_PORT GPIOA

void charging_init(void);
uint8_t is_charging(void);

void button_init(void);
uint8_t button_pressed(void);
void button_toggle(void);

extern void error_handler(void);

#ifdef __cplusplus
}
#endif

#endif
