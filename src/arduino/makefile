PROJ_NAME=ard1
CC=avr-g++
BD=build
CFLAGS=-c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -flto -fno-devirtualize -fno-use-cxa-atexit
BOARD_PARAMS=-mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10600 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR
SERIAL_PORT=/dev/ttyACM0
ARD_DIR=/usr/share/arduino/hardware/archlinux-arduino/avr
ARDUINO_INC=-I${ARD_DIR}/cores/arduino -I${ARD_DIR}/variants/standard
CUST_INC=-Iinclude -I../include

# Preprocessing of main file
${BD}/ard1.ino.cpp:ard1.ino
	${CC} ${CFLAGS} -x c++ -E -CC ${BOARD_PARAMS} ${ARDUINO_INC} ${CUST_INC} ard1.ino -o $@

${BD}/ard1.ino.cpp.o:${BD}/ard1.ino.cpp
	${CC} ${CFLAGS} -MMD ${BOARD_PARAMS} ${ARDUINO_INC} $< -o $@

${BD}/ard1.ino.elf:${BD}/ard1.ino.cpp.o core.a
	avr-gcc -w -Os -g -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=atmega328p $^ -lm -o $@

${BD}/ard1.ino.eep:${BD}/ard1.ino.elf
	avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0  ${BD}/ard1.ino.elf ${BD}/ard1.ino.eep

${BD}/ard1.ino.hex:${BD}/ard1.ino.elf
	avr-objcopy -O ihex -R .eeprom $^ $@

upload:${BD}/ard1.ino.hex
	sudo avrdude -C/etc/avrdude.conf -v -patmega328p -carduino -P${SERIAL_PORT} -D "-Uflash:w:$<:i"

clean:
	rm build/*

.DEFAULT_GOAL := upload
