/** The ADCs measures the current / voltage of the motors and the batteries.
 * The left motor is hooked up to ADC3 Channel 10. (DMA2CH5, PC0)
 * The right motor is hooked up to ADC1 Channel 11. (DMA1CH1, PC1)
 * The battery ADC1 Channel 12. (DMA1CH1, PC2)
 * The channels are alternated in measurement.
 */

#ifndef __ADC__H
#define __ADC__H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define ADC_BATTERY_VOLT     0.02647435897435897435897435897436
#define ROLLING_SAMPLES 16

struct ADC_setup {
	ADC_HandleTypeDef hadc;
	DMA_HandleTypeDef hdma_adc;
	uint32_t channel;

	IRQn_Type DMA_Channel_IRQn;
	int conversions;
};

struct ADC {
	struct ADC_setup setup;
	uint16_t motor_center;
	volatile uint16_t avg_current;
	volatile uint16_t data[2];
};

// ----------------------PUBLIC----------------------
void adcs_setup_and_init(void);

// ------------ROLLING_AVG----------------
float get_battery_volt(void);
float get_motor_current(struct ADC *adc);

extern void error_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */
