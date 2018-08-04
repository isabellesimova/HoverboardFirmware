/** This file includes the config constants.
 */

#ifndef __CONFIG__H
#define __CONFIG__H

// ----------------------MODES----------------------

/* Use CALIBRATION mode BEFORE running the firmware not in CALIBRATION mode
 * to figure out what the wheel offsets are. Since every wheel is different,
 * calibration is necessary to move the wheel smoothly. The process involves
 * slowly ramping up the power until the wheels start moving. Both wheels are done
 * in both directions, and the process repeats until it is stopped and reprogrammed.
 * You'll need to have UART (9600 baud rate) set up to see what the st microcontroller
 * is outputting.
 */
//#define CALIBRATION //comment out when not in use

/* Use DEBUG mode to send extra data via UART (such as the motor currents)
 */
#define DEBUG //comment out when not in use
/* Disable ADC debuging when in DEBUG mode, temporary fix for issue #2
   https://github.com/isabellesimova/HoverboardFirmware/issues/2
*/
#define DEBUG_NO_ADC //comment out when not in use

/* One beep at start
 */
//#define BUZZER_START_DEBUG //comment out when not in use

/* Pick a mode for control - trapezoidal or sinusoidal. Trapezoidal is simpler
 * and works with less logic. Sinusoidal is more complex but works better at low speeds.
 */
#define TRAPEZOIDAL_CONTROL 0
#define SINUSOIDAL_CONTROL 1
#define CONTROL_METHOD TRAPEZOIDAL_CONTROL




// --------------- WHEEL RELAETD ---------------
/* WHEEL CONSTANTS
 * Ticks in a revolution. Most Hoverboard wheels have 90 in a revolution.
 */
#define WHEEL_HALL_COUNTS 		90

/* WHEEL SETUP
 * The _OFFSET constants are sent via UART when the microcontroller runs the
 * firmware in CALIBRATION mode.
 * The _DIR offsets represent whether a positive rpm value should drive the wheel
 * clockwise or counterclockwise, where +1 is CCW and -1 is CW.
 * The current setup has it going forward if  you give both wheels
 * positive speeds.
 */
#define L_POS_OFFSET 		5
#define L_NEG_OFFSET 		2
#define L_WHEEL_DIR 		1
#define R_POS_OFFSET 		5
#define R_NEG_OFFSET		2
#define R_WHEEL_DIR 	   -1





/* SAFETY LIMIT
 * The wheel at max power goes REALLY fast. Most speeds can be achieved with <25%
 * the full power. Set a maximum of the power any wheel should reach.
 */
#define MAX_POWER_PERCENT 20

/* CHECKING FREQUENCIES
 * The frequencies for various tasks - power hcecks, UART communication, heart beat check.
 */
#define POWER_CHECK_PERIOD 5000 //ms
#define TX_WAIT_PERIOD	  250   //ms
#define RX_WAIT_PERIOD    50    //ms
#define HEARTBEAT_PERIOD  500   //ms

#endif /* __CONFIG__H */
