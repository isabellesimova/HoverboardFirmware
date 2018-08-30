# Hoverboard Motor Controller

This repository contains firmware to reflash the motor controller that comes with hoverboards. The new firmware allows us to control the wheels by commanding the speed (in rpm, rotations per minute) for each wheel via UART.

## Setup
1. [Setup the hardware](doc/1_HardwareSetup.md) - connect the lines needed to communicate with the microcontroller on the hoverboard motor controller board to the ST-LINK.
2. [ST-Link communication setup](doc/2_STLinkSetup.md) - make sure you can detect and communicate with the motor controller board from your computer via the ST device.
3. [Configure Hoverbot](doc/3_Configuration.md) - calibrate the wheels (via flashing the microcontroller) and look through other configuration settings.
4. [Run Hoverbot](doc/4_RunningHoverbot.md) - flash the microcontroller with the code that makes the bot move!

Other setup guides:
- [Debug with SystemWorkbench](doc/SystemWorkbenchSetup.md)




## Acknowledgments
Inspiration for the orginal version of this code can be attributed to [NiklasFauth/Hoverboard-Board-Hack](https://github.com/NiklasFauth/Hoverboard-Board-Hack).

## License

This project is licensed under the MIT License.
