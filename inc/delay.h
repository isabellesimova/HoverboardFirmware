/** This file deals with delays. */

#ifndef __DELAY_H
#define __DELAY_H

#define DELAY_TIM_FREQUENCY_US 1000000		/* = 1MHZ -> timer runs in microseconds */
#define DELAY_TIM_FREQUENCY_MS 1000			/* = 1kHZ -> timer runs in milliseconds */

void delay_ms(uint32_t mSecs);
void delay_us(uint32_t uSecs);

#endif
