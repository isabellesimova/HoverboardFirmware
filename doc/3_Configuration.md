# Configuring Hoverbot
## Flashing the firmware
Now that everything is set up, it's time to actually flash some new firmware onto the device!

### Dependencies
In order to compile the firmware you'll need to install the [GNU Embedded Toolchain for Arm](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads). You can download and install the toolchain appropriate for your development machine or use your favorite package manager. Below are instructions for using homebrew on MacOS.

#### MacOS
```
$ brew tap PX4/homebrew-px4
$ brew update
$ brew install gcc-arm-none-eabi
```

#### Linux
```
$ sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
$ sudo apt-get update
$ sudo apt-get install gcc-arm-embedded
$ sudo ln -s /usr/bin/arm-none-eabi-* /usr/local/bin/
```

## Calibration
### Configuration and Flashing
Before we can flash the code that moves the motors, there are some values we need to configure. If you open up the [config file](../inc/config.h), you'll see a bunch of things that can be configured. Right now, we just need to turn on calibration mode -- uncomment the line that defines CALIBRATION.
```
#define CALIBRATION //comment out when not in use
```

Now that that's set up, we can flash the calibration firmware by:
1. cd-ing to your repo (the directory containg the inc/ and src/ directories and the Makefile).
2. Compile the files by typing
```
$ make
```
3. Flash our device!

```
$ st-flash write build/hoverboard.bin 0x8000000
```

If it works successfully, you'll get a message that ends with:

```
INFO src/common.c: Starting verification of write complete
INFO src/common.c: Flash written and verified! jolly good!
```

Note that if you haven't connected the reset pin, and get the following error, you may need to try it a few times.
```
INFO src/flash_loader.c: Successfully loaded flash loader in sram
  x/12 pages written
ERROR src/flash_loader.c: flash loader run error
ERROR src/common.c: stlink_flash_loader_run(0x8003000) failed! == -1
stlink_fwrite_flash() == -1
```

### Setting up UART
Before we turn everything on, in order to see what the ST is outputting, the UART pins need to set up. The three pins you need are located on the top right side of the board (right most 4 pin connector/harness), just next to the speaker. We'll be using the green (RX), blue (TX), and black (GND) pins - Note that the red pin is 14V and we do NOT want to use it. Hook the three pins to any device with 3.3V UART, and configure the UART to run at a baud rate of 9600. (Remember that TX (blue) on the board should go to RX of the other device and RX (green) on the board should go to TX of the other device). Please note that some boards have wire colors that do not match as described, so it is best to check. The order from left to right on the connector is GND, RX, TX, 14V.

If your device is a raspberry pi, make sure to enable serial in raspi-config and restart your pi. Then, you can use a program such as minicom. (You may need to apt-get install minicom first):
```
pi@raspberrypi:~ $minicom -b 9600 -D /dev/ttyS0
```
(Note that to exit out of minicom, you type Ctrl+A, then Q.)

Now that we have the UART set up, we can see what the configuration settings for the wheels should be. Turn on the motor board and wait for the wheels to slowly start moving - it can take a bit for the wheels to start moving. You should see messages of the format (which will repeat until turned off):

```
L+1:#
L-1:#
R+1:#
R-1:#
```

### Finish Configuration
Now that we have the wheel offsets, we can finish the configuration file. Where it says WHEEL SETUP are a bunch of constants you need to fill in.
```
#define L_POS_OFFSET    5 // Number from L+1
#define L_NEG_OFFSET    2 // Number from L-1
#define L_WHEEL_DIR     1
#define R_POS_OFFSET    5 // Number from R+1
#define R_NEG_OFFSET    2 // Number from R-1
#define R_WHEEL_DIR     -1
```
Note that the positive and negative offsets should be off by 3 for either side - if it's not 3, try running the calibration mode setup longer, as it will occasionally be off by one.

The other constants in the [config file](../inc/config.h) should be set at this time as well. For a generic use case, these can be left with their default values.
1. DEBUG mode - uncomment this if you want extra measurements over UART
2. CONTROL_METHOD - should be either SINUSOIDAL or TRAPEZOIDAL
3. WHEEL_HALL_COUNTS - should be 90 for hoverboard wheels
4. MAX_POWER_PERCENT - the max power percent any wheel should reach
5. POWER_CHECK_PERIOD - how often battery voltage and charging are checked
6. TX_WAIT_PERIOD - how often to transmit measurements
7. RX_WAIT_PERIOD - how often to process UART chars received
8. HEARTBEAT_PERIOD - how often the board is expecting commands in ms (should be increased if manually typing commands to drive wheels)
