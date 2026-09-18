#!/bin/bash

# 1. Compile the C code into an ELF executable
avr-gcc -g -Os -mmcu=atmega32 -o hello.elf hello.c

# 2. Extract the executable code and data to generate the HEX file
avr-objcopy -j .text -j .data -O ihex hello.elf hello.hex

# 3. Flash the HEX file to the ATmega32 using USBasp
avrdude -c usbasp -p m32 -U flash:w:hello.hex:i