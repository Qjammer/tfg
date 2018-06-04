ifeq (${MAKECMDGOALS},upload-ArdUNO)
PROJ_NAME:=ArdUNO
CHIPSET:=atmega328p
MACRO_DEFS=-DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR
BOARD_INC=-I${ARD_DIR}/variants/standard
PROGRAMMER=arduino
CHIP_FREQ:=16000000L
SERIAL_PORT:=/dev/ttyArdUNO
CU:=ArdUNO.cpp
LIBS_CPP:=LIDARLite.cpp Wire.cpp twi.c Servo.cpp
endif

ifeq (${MAKECMDGOALS},upload-ArdMEGA)
PROJ_NAME:=ArdMEGA
CHIPSET:=atmega2560
MACRO_DEFS=-DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR
BOARD_INC=-I${ARD_DIR}/variants/mega
PROGRAMMER=wiring
CHIP_FREQ:=16000000L
SERIAL_PORT:=/dev/ttyArdMEGA
CU:=ArdMEGA.cpp Motor.cpp
LIBS_CPP:=Wire.cpp twi.c Servo.cpp TimerOne.cpp PID_v1.cpp
endif

include make_arduino

CUST_INC=-Iinclude -I../include

# Compile units
${BD}/%.cpp.o:%.cpp
	${CP} ${CPPFLAGS} ${BOARD_PARAMS} ${ARDUINO_INC} ${ARDLIB_INC} ${CUST_INC} $< -o $@

