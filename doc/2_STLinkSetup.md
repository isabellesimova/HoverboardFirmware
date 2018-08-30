# ST Link Setup

Now that everything is wired up, let's make sure we can communicate between our computer and the ST chip on the microcontroller. If you don't already have the ST-link tools, you can install it by following the instructions for your operating system  on the README of the [Stlink Tools Repo](https://github.com/texane/stlink#installation).

#### Linux:
Following [the installation guide](https://github.com/texane/stlink/blob/master/doc/compiling.md):
```
$ sudo apt-get install cmake
$ sudo apt-get install libusb-1.0.0
$ git clone https://github.com/texane/stlink.git
$ cd stlink
$ make release
$ cd build/Release; sudo make install
$ sudo ldconfig
```

## Using ST link

Now that you have the tools ready, let's probe our ST link to see what it's detecting:
```
$ st-info --probe
```
In response, we should see something like:
```
Found 1 stlink programmers
 serial: 563f7206513f52504832153f
openocd: "\x56\x3f\x72\x06\x51\x3f\x52\x50\x48\x32\x15\x3f"
  flash: 262144 (pagesize: 2048)
   sram: 65536
 chipid: 0x0414
  descr: F1 High-density device
```

If you have a device show up, we're ready to move on to the [next section](2_STLinkSetup.md#unlocking-the-device)! Otherwise, feel free to check out some common problems:

### Troubleshooting
- If you see:
```
Found 1 stlink programmers
```
This means that the ST Link is detected, but it can't find the ST. This usually means that the TVCC pin isn't getting 3.3V or the ST isn't getting 3.3V. Make sure your ground and power pins are hooked up correctly, as detailed in the previous section (ie, for the larger ST Link, pin 19 -- pin 1, and pin 2 -- ST 3.3V).
- If you see:
```
Found 0 stlink programmers
```
This generally means that your computer couldn't find the ST Link V2. Make sure your ST Link V2 is connected to your computer and wired up correctly. You can also try unplugging it and replugging it in. If it still is not detected, try restarting your computer to reset your serial ports. If that still does not work, you might have a borked ST Link V2, and you should acquire another one.

## Unlocking the device

A lot of the STs come locked for protection. We can unlock it (which erases the code on it) using openocd (which is installed with ST Link Tools):
```
$ openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "stm32f1x unlock 0" -c "reset halt" -c "exit"
```

In response, you should see a bunch of INFO messages, as well as:
```
stm32x unlocked.
```
at some point in the messages. If you see the unlocked message, we're ready to move on. If you can't unlock it, you might need to find a device with windows OS, and use the ST Link tools to unlock it.