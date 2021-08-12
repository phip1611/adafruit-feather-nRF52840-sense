# Adafruit Feather nRF52840 Sense

Board by Adafruit with nRF52840 as µC and Bluetooth 5.2 SoC (plus a few other wireless technologies).
The nRF52840 is built around the 32-bit ARM® Cortex™-M4 CPU with floating point unit running at 64 MHz.

More info: https://www.nordicsemi.com/Products/nRF52840

It contains several sensors and the official documentation is here:

https://learn.adafruit.com/adafruit-feather-sense

The description of the sensors is here:

https://learn.adafruit.com/adafruit-feather-sense/pinouts

Full board spec:

https://learn.adafruit.com/adafruit-feather-sense/downloads

## Programming in General

All sensors are connected via internal I2C bus.

### C
I got it working using the "PlatformIO" plugin for Visual Studio Code + the native installation of PlatformIO.
PlatformIO takes care of locating the board via USB, uploading code to it and connecting the serial output 
to the terminal. It is really straight forward once you understand the PlatformIO project setup and 
workflow. It might be necessary to open the C-directory directly in VS Code and not this "super directory".

For each of the sensors there is already an official adafruit library/driver on github which can be included
really easy. They have example code there and the default I2C address is already defined by almost every
driver. 

### Rust
Todo, not working yet.
