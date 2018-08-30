/** Basic UART communication with whatever is hooked 
 * up to the long UART wires (closer to the speaker).
 */

#include "uart.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "constants.h"

// ----------------------PRIVATE----------------------
static int process_frame(uint8_t start, uint8_t length);

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

volatile struct UART uart;

extern volatile int8_t status;
extern uint16_t speeds[2];

/* USART2 init function
 * BAUD RATE is 9600.
 */
void MX_USART2_UART_Init(void)
{
	__HAL_RCC_USART2_CLK_ENABLE();
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;

	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		error_handler();
	}

	uart.RX_pointer = 0;
	uart.RX_available = 1;
	uart.TX_free = 1;

	HAL_UART_Receive_DMA(&huart2, (uint8_t*) uart.RX_buffer, BUFFER_LENGTH);
}

/* Look at the bytes you haven't processed yet, and using SLIP, process each non-empty frame.
 * SLIP end / escape characters are defined in constants.h
 */
uint8_t Uart_RX_process() {

	int8_t i, rx_length;
	uint8_t dma_count, cur_index, rx_value;
	uint8_t cur_frame_find, last_frame_find, frame_length;
	uint8_t next_RX_pointer;

	if (uart.RX_available) {

		//find the length of received data
		dma_count =  BUFFER_LENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		rx_length = dma_count - uart.RX_pointer; //difference from last count

		if (rx_length < 0) {
			rx_length = (rx_length + BUFFER_LENGTH) % BUFFER_LENGTH; //if wrapped around, fix the length
		} else if (rx_length == 0) {
			return 0;
		}

		//process the characters backward and find the last valid command
		//expected format: \nL###,R###\r\n <-- some combination of newline characters
		//check new line characters for framing

		cur_frame_find = 0;
		last_frame_find = 0;
		next_RX_pointer = 0xFF; //invalid, because buffer is length 128

		for (i = rx_length - 1; i >= 0; i--) {
			cur_index = (i + uart.RX_pointer) % BUFFER_LENGTH;
			rx_value = uart.RX_buffer[cur_index];

			//no matter what reset parsing on any new line character
			if (rx_value == FRAME_END1 || rx_value == FRAME_END2) {
				if (next_RX_pointer == 0xFF) { //most recent newline character hasn't been found yet
					last_frame_find = cur_index;
					next_RX_pointer = (cur_index - 1 + BUFFER_LENGTH) % BUFFER_LENGTH;
				} else {
					cur_frame_find = cur_index;
					frame_length = (last_frame_find - cur_frame_find - 1 + BUFFER_LENGTH) % BUFFER_LENGTH;
					last_frame_find = cur_frame_find;
					if (frame_length > 0) {
						if (process_frame((cur_index+1) % BUFFER_LENGTH, frame_length) == 1) {
							uart.RX_pointer = next_RX_pointer;
							return 1;
						}
					}
				}
			}
		}

		if (next_RX_pointer != 0xFF) {
			uart.RX_pointer = next_RX_pointer;
		}
	}

	return 0;
}

/* Do the actual transmission of data.
 * NEED TO CHECK IF TX IS FREE BEFORE CALLING THIS METHOD.
 */
void Uart_TX(char *message)
{
	uart.TX_free = 0;    //busy
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)message, strlen(message));
}

/* Check if RX line is free -- flag is cleared upon RX Completion.
 */
uint8_t Uart_is_RX_available(void){
	return (uart.RX_available);
}

/* Check if TX line is free -- flag is cleared upon TX Completion.
 */
uint8_t Uart_is_TX_free(void){
	return (uart.TX_free);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
		uart.RX_available = 1;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		uart.TX_free = 1;
	}
}


// ----------------------PRIVATE----------------------
/* Process a single SLIP frame. It should be of the format:
 * "L(-)##(#),R(-)##(#)"
 * If the format is not correct, then the bad parsing flag is set.
 * Once a correct frame is parsed, the bad parsing flag will be cleared.
 */
static int process_frame(uint8_t start, uint8_t length) {
	int8_t i = 0, rx_value = 0;
	int8_t parse_state = 0;
	int8_t digits_first = 1, rx_digits = 0;
	int16_t rx_sign = 1, l_sign = 1;
	int16_t rx_speed = 0, l_speed = 0;

	//frame should match format "L(-)##(#),R(-)##(#)"

	//parse states:
	// 0: expect an L
	// 1: expect a "-" or "#" or ","
	// 2: expect an R
	// 3: expect a "-" or "#" or ","

	for (i = 0; i < length; i++) {

		rx_value = uart.RX_buffer[(i + start) % BUFFER_LENGTH];

		//SLIP escaping
		if (rx_value == FRAME_ESC) {
			i = i+ 1;
			if (i >= length) {
				SET_ERROR_BIT(status, STATUS_BAD_PARSING);
				return 0;
			}

			rx_value = uart.RX_buffer[(i + start) % BUFFER_LENGTH];
			if (rx_value == ESC_END1) {
				rx_value = FRAME_END1;
			} else if (rx_value == ESC_END2) {
				rx_value = FRAME_END2;
			} else if (rx_value == ESC_ESC) {
				rx_value = FRAME_ESC;
			} else {
				SET_ERROR_BIT(status, STATUS_BAD_PARSING);
				return 0;
			}
		}

		// state parsing
		if (parse_state % 2 == 0) { //expect L or R
			if ((parse_state == 0 && rx_value == 'L') || (parse_state == 2 && rx_value == 'R')) {
				parse_state = parse_state + 1;
				rx_speed = 0;
				rx_sign = 1;
				digits_first = 1;
				rx_digits = 0;
			} else {
				SET_ERROR_BIT(status, STATUS_BAD_PARSING);
				return 0;
			}
		} else { // expect a "-" or "#" or ","
			if (digits_first == 1 && rx_value == '-') {
				rx_sign = -1;
			} else if (rx_value >= '0' && rx_value <= '9' && rx_digits < 4){
				rx_speed = rx_speed * 10 + (rx_value - '0') ;
				rx_digits = rx_digits + 1;
			} else if (parse_state == 1 && rx_digits > 0 && rx_value == ',') { //end delimiter of "L" part
				parse_state = 2;
				l_speed = rx_speed;
				l_sign = rx_sign;
			} else {
				SET_ERROR_BIT(status, STATUS_BAD_PARSING);
				return 0;
			}
			digits_first = 0;
		}
	}

	if (parse_state == 3 && rx_digits > 0) {
		speeds[0] = l_speed * l_sign;
		speeds[1] = rx_speed * rx_sign;
		CLR_ERROR_BIT(status, STATUS_BAD_PARSING);
		return 1;
	} else {
		SET_ERROR_BIT(status, STATUS_BAD_PARSING);
		return 0;
	}
}
