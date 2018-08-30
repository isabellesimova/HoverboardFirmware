/** The ADCs measures the current / voltage of the motors and the batteries.
 * The left motor is hooked up to ADC3 Channel 10. (DMA2CH5, PC0)
 * The right motor is hooked up to ADC1 Channel 11. (DMA1CH1, PC1)
 * The battery ADC1 Channel 12. (DMA1CH1, PC2)
 * The channels are alternated in measurement.
 */

#include "adc.h"

struct ADC adc_L;
struct ADC adc_R;

static int battery_voltage;

// ----------------------PRIVATE----------------------
static void adc_init(struct ADC *adc);
static void adc_calibrate(struct ADC *adc);
static uint16_t adc_battery(void);
static uint16_t adc_motor(struct ADC *adc);

// ----------------------PUBLIC----------------------
/* Configure both ADCs and set them up
 */
void adcs_setup_and_init() {
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

/* Return a rolling average of the battery voltage (last 16 samples).
 */
float get_battery_volt(void) {
	//fixed point, everything * 16
	battery_voltage -= battery_voltage / ROLLING_SAMPLES;
	battery_voltage += adc_battery();
	return (float) (battery_voltage / ROLLING_SAMPLES) * ADC_BATTERY_VOLT;
}

/* Return a rolling average of the motor current (last 16 samples).
 */
float get_motor_current(struct ADC *adc) {
	//rolling average, fixed point, everything *16
	adc->avg_current -= adc->avg_current / ROLLING_SAMPLES;
	adc->avg_current += adc_motor(adc);
	return adc->avg_current / ROLLING_SAMPLES;
}

// ----------------------PRIVATE----------------------
/* Initialize the ADCs to convert continuously and start DMA to transfer
 * readings to memory.
 */
static void adc_init(struct ADC *adc) {

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
		battery_voltage = adc_battery() * ROLLING_SAMPLES;
	}
}

/* Calibrate the readings so that the rolling average
 * doesn't include readings before DMA starts transferring.
 */
static void adc_calibrate(struct ADC *adc) {
	//calibrate the avg
	int i;
	for (i = 0; i < ROLLING_SAMPLES; i++) {
		adc->avg_current += adc_motor(adc);
	}
	adc->motor_center = adc->avg_current / ROLLING_SAMPLES;
}

/* Retrieve the voltage of the battery from where DMA transferred it.
 */
static uint16_t adc_battery(void) {
	uint16_t data = 0;
	data = adc_R.data[1];
	if (data == 0)
		return adc_battery();
	return data;
}

/* Retrieve the voltage of the motor, which is proportional to the current.
 */
static uint16_t adc_motor(struct ADC *adc) {
	uint16_t data = 0;
	data = adc->data[0];
	if (data == 0)
		return adc_motor(adc);
	return data;
}

