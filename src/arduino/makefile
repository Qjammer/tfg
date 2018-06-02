PROJ_NAME=ardtest
COMPILE_UNITS=ardtest.cpp Motor.cpp
CUST_INC=-Iinclude -I../include

BD=build
CBD=corebuild
OBJECTS=${patsubst %,${BD}/%.o,${COMPILE_UNITS}}
CC=avr-gcc
CP=avr-g++
COMMONCFLAGS=-c -g -Os -Wall -Wextra -ffunction-sections -fdata-sections -flto
CFLAGS=${COMMONCFLAGS} -std=gnu11 -fno-fat-lto-objects
CPPFLAGS=${COMMONCFLAGS} -std=gnu++11 -fpermissive -fno-exceptions -fno-threadsafe-statics -fno-devirtualize -fno-use-cxa-atexit
CHIPSET=atmega328p
CHIP_FREQ=16000000L
BOARD_PARAMS=-mmcu=${CHIPSET} -DF_CPU=${CHIP_FREQ} -DARDUINO=10805 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR
SERIAL_PORT=/dev/ttyACM0
ARD_DIR=/usr/share/arduino/hardware/archlinux-arduino/avr
CORE_DIR=${ARD_DIR}/cores/arduino
CORE_CPP=${wildcard ${CORE_DIR}/*.cpp}
CORE_C=${wildcard ${CORE_DIR}/*.c}
CORE_ASM=${wildcard ${CORE_DIR}/*.S}
CORE_ALL=${CORE_CPP} ${CORE_C} ${CORE_ASM}
CORE_OBJ_TMP=${notdir ${CORE_ALL}}
CORE_OBJ=${patsubst %,${CBD}/%.o,${CORE_OBJ_TMP}}
ARDUINO_INC=-I${CORE_DIR} -I${ARD_DIR}/variants/standard
ARDUINO_LIBS=-I${ARD_DIR}/libraries

print-%:
	@echo '$*=$($*)'

# Core unit
${CBD}/core.a:${CORE_OBJ}
	avr-gcc-ar rcs $@ $^

# Core-cpp compile units
${CBD}/%.cpp.o:${CORE_DIR}/%.cpp
	${CP} ${CPPFLAGS} ${BOARD_PARAMS} ${ARDUINO_INC} $< -o $@

# Core-c compile units
${CBD}/%.c.o:${CORE_DIR}/%.c
	${CC} ${CFLAGS} ${BOARD_PARAMS} ${ARDUINO_INC} $< -o $@

# Core-asm compile units
${CBD}/%.S.o:${CORE_DIR}/%.S
	${CC} -c -g -x assembler-with-cpp -flto ${BOARD_PARAMS} ${ARDUINO_INC} $< -o $@



# Compile units
${BD}/%.cpp.o:%.cpp
	${CP} ${CPPFLAGS} ${BOARD_PARAMS} ${ARDUINO_INC} ${ARDUINO_LIBS} ${CUST_INC} $< -o $@

# Linking program
${BD}/${PROJ_NAME}.elf:${OBJECTS} ${CBD}/core.a
	avr-gcc -w -Os -g -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=${CHIPSET} $^ -lm -o $@

# EEPROM Data
${BD}/${PROJ_NAME}.eep:${BD}/${PROJ_NAME}.elf
	avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0  $< $@

#Preparing program for transmission
${BD}/${PROJ_NAME}.hex:${BD}/${PROJ_NAME}.elf
	avr-objcopy -O ihex -R .eeprom $^ $@

#Uploading file to arduino
upload:${BD}/${PROJ_NAME}.hex
	sudo avrdude -C/etc/avrdude.conf -v -p${CHIPSET} -carduino -P${SERIAL_PORT} -D "-Uflash:w:$<:i"
.PHONY=upload

clean:
	rm build/* corebuild/*
.PHONY=clean

.DEFAULT_GOAL := upload

