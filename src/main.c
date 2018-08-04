#include "stm32f1xx_hal.h"
#include "main.h"
#include "constants.h"
#include "config.h"
#include "power.h"
#include "debug.h"
#include "uart.h"
#include "adc.h"
#include "motor.h"


extern volatile __IO struct UART uart;


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void error_handler(void);
static void MX_IWDG_Init(void);
void receive_data();
void transmit_data();
void check_power();


IWDG_HandleTypeDef hiwdg;
#ifdef DEBUG
extern struct ADC adc_L;
extern struct ADC adc_R;
#endif

volatile uint32_t time, last_tx_time, last_rx_time, last_pwr_time;
volatile int8_t status;
int16_t speeds[2];

/* MAIN
 * Setup the clock and watchdog, and initialize all the peripherals.
 * Check the RX data, TX data, and power statuses at different intervals.
 */
int main(void)
{

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	/* Watchdog setup -- force restart if something unexpected happens*/
	MX_IWDG_Init();

	// power everything
	button_init();
	buzzer_init();
	#ifdef BUZZER_START_DEBUG
	buzzer_one_beep();
	#endif
	charging_init();
	led_init();
	led_set(1);

	MX_USART2_UART_Init();
	ADCs_setup_and_init();
	Motors_setup_and_init();


#ifdef CALIBRATION

	while (1) {
		Motors_calibrate();
	}
#else

	last_rx_time = HAL_GetTick();
	last_tx_time = HAL_GetTick();
	last_pwr_time = HAL_GetTick();

	while (1) {
		time = HAL_GetTick();

		// get the commands every RX_WAIT_PERIOD ms
		if (time - last_rx_time > RX_WAIT_PERIOD) {
			if (!CHK_ERROR_BIT(status, STATUS_LOW_BATTERY) && !CHK_ERROR_BIT(status, STATUS_IS_CHARGING)) {
				receive_data();
			}
		}

		// transmit status/info every TX_WAIT_PERIOD ms
		if (time - last_tx_time > TX_WAIT_PERIOD) {
			transmit_data();
		}

		// check to make sure power levels are ok
		if (time - last_pwr_time > POWER_CHECK_PERIOD) {
			check_power();
		}

		HAL_IWDG_Refresh(&hiwdg);   //819mS
	}

#endif

}


// ----------- PRIVATE METHODS ----------------
/* RECEIVE DATA
 * Process the newly received data. If a proper frame was processed, update the last_rx_time.
 * Update the motors to the new speeds.
 */
void receive_data() {
	int uart_rx_status = Uart_RX_process();
	if (uart_rx_status == 1) {
		last_rx_time = HAL_GetTick();
		Motors_speeds(speeds[0], speeds[1]);
	}
}

/* TRANSMIT DATA
 * Send the status byte, as well as the battery voltage if the TX line is free.
 * In DEBUG mode, additional readings are outputted - current readings from the wheel.
 */
void transmit_data() {
	float data_v;
	data_v = GET_BATTERY_VOLT();

#if defined(DEBUG) && !defined(DEBUG_NO_ADC)
	//TODO: These readings are not in amps, needs work.
	float data_i_L, data_i_R;
	data_i_L = GET_MOTOR_AMP(&adc_L);
	data_i_R = GET_MOTOR_AMP(&adc_R);
	sprintf((char *)&uart.TX_buffer[0],"[%d, %d, %d, %d]\n", status, (int)data_v, (int)data_i_L, (int)data_i_R);
#else
	sprintf((char *)&uart.TX_buffer[0],"[%d, %d]\n", status, (int)data_v);
#endif

	if (Uart_is_TX_free()) {
		Uart_TX((char *)&uart.TX_buffer[0]);
		last_tx_time = HAL_GetTick();
	}
}

/* Check two power related things:
 * - the battery level is too low (limit set at 32V)
 * - if it's charging
 *
 * If either of them is true, the robot should not move,
 * and the corresponding error bits should be set in the status byte.
 */
void check_power() {
	/* based off of power button at PA1
	 * PA1 is detected high at ~2V in 3.3V system
	 * voltage detected is 1/16 of battery voltage
	 */
	if (GET_BATTERY_VOLT() < 32) {
		SET_ERROR_BIT(status, STATUS_LOW_BATTERY);
		Motors_stop();
		buzzer_short_beep();
	} else {
		CLR_ERROR_BIT(status, STATUS_LOW_BATTERY);
	}
	HAL_IWDG_Refresh(&hiwdg);   //819mS


	/* don't move if we are charging
	 */
	if (is_charging()) {
		SET_ERROR_BIT(status, STATUS_IS_CHARGING);
		Motors_stop();
	} else {
		CLR_ERROR_BIT(status, STATUS_IS_CHARGING);
	}

	last_pwr_time = HAL_GetTick();
}


/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		error_handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		error_handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		error_handler();
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{
	__HAL_RCC_WWDG_CLK_ENABLE();
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		error_handler();
	}
	HAL_IWDG_Start(&hiwdg);
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void error_handler(void)
{
	/* USER CODE BEGIN error_handler */
	/* User can add his own implementation to report the HAL error return state */
	Motors_stop();

	while(1)
	{
	}
	/* USER CODE END error_handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
