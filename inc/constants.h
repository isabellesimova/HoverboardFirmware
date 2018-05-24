/** This file includes the constants.
 */

#ifndef __CONSTANTS__H
#define __CONSTANTS__H

/* ERROR BITS
 * Defines what each bit in the status byte represents
 */
#define STATUS_BAD_PARSING		 	0
#define STATUS_SPEED_OUT_OF_BOUNDS 	1
#define STATUS_HEARTBEAT_MISSING	2
#define STATUS_MAX_POWER_REACHED    3
#define STATUS_LOW_BATTERY		 	4
#define STATUS_IS_CHARGING		 	5
#define STATUS_UNKNOWN_1			6 //can set this to any other error status
#define STATUS_UNKNOWN_2    		7 //can set this to any other error status

/* BIT OPERATOINS
 * Methods to set, clear, or check if an error bit is set.
 */
#define SET_ERROR_BIT(p,n) ((p) |= (1 << (n)))
#define CLR_ERROR_BIT(p,n) ((p) &= ~((1) << (n)))
#define CHK_ERROR_BIT(p,n) ((p >> n) & 1)

/* UART SLIP constants
 * UART is implemented with SLIP, but with different end frame characters to make
 * it more user friendly. Instead, there are two possible end frame constants
 * (either counts as end frame) - this helps with the different return characters
 * different operating systems use.
 */
#define FRAME_END1  '\n'
#define FRAME_END2  '\r' // if only one frame end character desired, mark them as the same thing
#define FRAME_ESC    0xDB //STANDARD
#define ESC_END1     0xDC //STANDARD
#define ESC_END2     0xDE
#define ESC_ESC      0xDD //STANDARD

#endif /* __CONSTANTS_H */
