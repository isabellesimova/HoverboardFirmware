/** The ADCs measures the current / voltage of the motors and the batteries.
 * The left motor is hooked up to ADC3 Channel 10. (DMA2CH5, PC0)
 * The right motor is hooked up to ADC1 Channel 11. (DMA1CH1, PC1)
 * The battery ADC1 Channel 12. (DMA1CH1, PC2)
 * The channels are alternated in measurement.
 */

#include "adc.h"

struct ADC adc_L;
struct ADC adc_R;

int battery_voltage;

// ----------------------PUBLIC----------------------
/* Configure both ADCs and set them up
 */
void ADCs_setup_and_init() {
	adc_R.setup.hadc.Instance = ADC1;
	adc_R.setup.DMA_Channel_IRQn = DMA1_Channel1_IRQn;
	adc_R.setup.channel = ADC_CHANNEL_11;
	adc_R.setup.conversions = 2;

	adc_L.setup.hadc.Instance = ADC3;
	adc_L.setup.DMA_Channel_IRQn = DMA2_Channel4_5_IRQn;
	adc_L.setup.channel = ADC_CHANNEL_10;
	adc_L.setup.conversions = 1;

	adc_init(&adc_R);
	adc_init(&adc_L);
}

/* Initialize the ADCs to convert continuously and start DMA to transfer
 * readings to memory.
 */
void adc_init(struct ADC *adc) {

	ADC_ChannelConfTypeDef sConfig;

	if (adc->setup.hadc.Instance == ADC1) { // right ADC
		__HAL_RCC_DMA1_CLK_ENABLE();
	} else if (adc->setup.hadc.Instance == ADC3) {
		__HAL_RCC_DMA2_CLK_ENABLE();
	}

	adc->setup.hadc.Init.ScanConvMode = ADC_SCAN_ENABLE;
	adc->setup.hadc.Init.ContinuousConvMode = ENABLE;
	adc->setup.hadc.Init.DiscontinuousConvMode = DISABLE;
	adc->setup.hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	adc->setup.hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	adc->setup.hadc.Init.NbrOfConversion = adc->setup.conversions; // right + battery
	if (HAL_ADC_Init(&(adc->setup.hadc)) != HAL_OK) {
		error_handler();
	}

	// Configure channel for the motor
	sConfig.Channel = adc->setup.channel;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&(adc->setup.hadc), &sConfig) != HAL_OK) {
		error_handler();
	}

	if (adc->setup.conversions == 2) {
		// Configure channel for the battery
		sConfig.Channel = ADC_CHANNEL_12;
		sConfig.Rank = 2;
		if (HAL_ADC_ConfigChannel(&(adc->setup.hadc), &sConfig) != HAL_OK) {
			error_handler();
		}
	}

	HAL_ADC_Start_DMA(&(adc->setup.hadc), (uint32_t*) adc->data,
			adc->setup.conversions); //right + battery

	adc_calibrate(adc);

	if (adc->setup.conversions == 2) {
		battery_voltage = ADC_BATTERY() * ROLLING_SAMPLES;
	}
}

/* Calibrate the readings so that the rolling average
 * doesn't include readings before DMA starts transferring.
 */
void adc_calibrate(struct ADC *adc) {
	//calibrate the avg
	int i;
	for (i = 0; i < ROLLING_SAMPLES; i++) {
		adc->avg_current += ADC_MOTOR(adc);
	}
	adc->motor_center = adc->avg_current / ROLLING_SAMPLES;
}

// ------------ROLLING AVG----------------
/* Return a rolling average of the battery voltage (last 16 samples).
 */
float GET_BATTERY_VOLT(void) {
	//fixed point, everything * 16
	battery_voltage -= battery_voltage / ROLLING_SAMPLES;
	battery_voltage += ADC_BATTERY();
	return (float) (battery_voltage / ROLLING_SAMPLES) * ADC_BATTERY_VOLT;
}

/* Return a rolling average of the motor current (last 16 samples).
 */
float GET_MOTOR_AMP(struct ADC *adc) {
	//rolling average, fixed point, everything *16
	adc->avg_current -= adc->avg_current / ROLLING_SAMPLES;
	adc->avg_current += ADC_MOTOR(adc);
	return adc->avg_current / ROLLING_SAMPLES;
}

// ------------RAW----------------
/* Retrieve the voltage of the battery from where DMA transferred it.
 */
uint16_t ADC_BATTERY(void) {
	uint16_t data = 0;
	data = adc_R.data[1];
	if (data == 0)
		return ADC_BATTERY();
	return data;
}

/* Retrieve the voltage of the motor, which is proportional to the current.
 */
uint16_t ADC_MOTOR(struct ADC *adc) {
	uint16_t data = 0;
	data = adc->data[0];
	if (data == 0)
		return ADC_MOTOR(adc);
	return data;
}
