# Head Tracker STM32F411 + LSM6DS3

The head tracker is based on LSM6DS3, an accelerometer and gyroscope sensor.
The microcontroller (STM32F411) interfaces with the sensor through SPI and sends processed data to a computer by USB.

The device identifies itself to the USB host (a computer) as an HID gamepad with two axes (X and Y) and one button.
No USB drivers need to be installed manually as most operating systems ship with the HID driver.

The program for the microcontroller was written entirely bare-metal (including USB logic), without using the STM32 HAL library.


## requirements
- GNU Arm Embedded Toolchain (added to Path)
- Makefile (added to Path)
- STM32 ST-Link Utility (ST-LINK_CLI.exe, added to Path)


## build
```
$ make all
```

## run
```
$ make flash_mcu
```