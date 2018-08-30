# Run Hoverbot!
## Flash Firmware to Move
Now that everything is set up, it's time to flash the board to move at our command!
1. First, let's do some clean up because we've changed some of the values in our header files. If we recompile, we'll have the firmware we want to use.
```
$ make clean ; make
```
2. Flash our device!

```
$ st-flash write build/hoverboard.bin 0x8000000
```

3. Done! Feel free to turn it on by pressing the on/off button.

## Let's roll!
Now that the firmware is set up, it's time to command the wheels to move! The ST is expecting commands over UART at a baud rate of 9600. Since the UART communication implements a variation of SLIP, commands should be **prefixed** and **postfixed** with any of the newline characters: \r or \n or \r\n.


The command format is
```
L[rpm],R[rpm]
```
where rpm stands for rotations per minute, and is limited to the range ±12 to ±360.

More specifically,
```
L(-)##(#),R(-)##(#)
```

Some examples of appropriate commands are:
```
- L20,R20
- L12,R-36
- L-31,R-200
```

As a safety precaution, a heartbeat has been included (configurable in the [config file](../inc/config.h)). This means that the wheels will stop moving if it has not received any new commands in however many milliseconds specified in the config file.

## Errors!
If the status byte you are receiving back is not 0... that usually means something is wrong.
```
BAD_PARSING           0 (the latest SLIP frame format is wrong)
SPEED_OUT_OF_BOUNDS   1 (the rpm wasn't between ±12 and ±360)
HEARTBEAT_MISSING     2 (the most recent command is over x ms old)
MAX_POWER_REACHED     3 (the user max threshold has been reached)
LOW_BATTERY           4 (the battery voltage is less than 32V)
IS_CHARGING           5 (the battery is charging)
```

Note that the motor board will also start beeping when the voltage is too low.