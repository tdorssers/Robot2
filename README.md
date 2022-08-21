# Robot2

Line following and collision avoidance robot version 2

## Overview

* ATmega32 @ 16 MHz
* HC-SR04 Ultrasonic Ranging Modules
* SG90 Servo
* L293D Motor Driver
* TCRT5000 Reflective Optical Sensors
* ITR9608 Optical Interrupters
* ILI9341 TFT LCD Driver
* QMC5883L Magnetic Sensor
* VL53L0X Time-of-Flight Ranging Sensors

## Schematic

![](schematic/Robot2.png)

## Prototype

![](media/Prototype.jpg)

## Firmware
The firmware has been developed in Atmel Studio 7 using GCC C and can be uploaded to the ATmega32 using the ISP connector and an ISP programmer such as [USBasp tool](http://www.fischl.de/usbasp/) using [avrdude](http://www.nongnu.org/avrdude/):

`avrdude -p m32 -c usbasp -U flash:w:Robot2.hex:i -U eeprom:w:Robot2.eep:i -U hfuse:w:0xC9:m -U lfuse:w:0xFF:m`
