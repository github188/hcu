################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/l2frame/api3g.c \
../src/l2frame/apigpio.c \
../src/l2frame/apigprs.c \
../src/l2frame/apigps.c \
../src/l2frame/apiserial.c \
../src/l2frame/apiwifi.c \
../src/l2frame/audio.c \
../src/l2frame/ble.c \
../src/l2frame/camera.c \
../src/l2frame/cloudcont.c \
../src/l2frame/con3g4g.c \
../src/l2frame/ethernet.c \
../src/l2frame/gps.c \
../src/l2frame/hwinv.c \
../src/l2frame/lcd.c \
../src/l2frame/modbus.c \
../src/l2frame/sps232.c \
../src/l2frame/sps485.c \
../src/l2frame/usbnet.c \
../src/l2frame/wifi.c 

OBJS += \
./src/l2frame/api3g.o \
./src/l2frame/apigpio.o \
./src/l2frame/apigprs.o \
./src/l2frame/apigps.o \
./src/l2frame/apiserial.o \
./src/l2frame/apiwifi.o \
./src/l2frame/audio.o \
./src/l2frame/ble.o \
./src/l2frame/camera.o \
./src/l2frame/cloudcont.o \
./src/l2frame/con3g4g.o \
./src/l2frame/ethernet.o \
./src/l2frame/gps.o \
./src/l2frame/hwinv.o \
./src/l2frame/lcd.o \
./src/l2frame/modbus.o \
./src/l2frame/sps232.o \
./src/l2frame/sps485.o \
./src/l2frame/usbnet.o \
./src/l2frame/wifi.o 

C_DEPS += \
./src/l2frame/api3g.d \
./src/l2frame/apigpio.d \
./src/l2frame/apigprs.d \
./src/l2frame/apigps.d \
./src/l2frame/apiserial.d \
./src/l2frame/apiwifi.d \
./src/l2frame/audio.d \
./src/l2frame/ble.d \
./src/l2frame/camera.d \
./src/l2frame/cloudcont.d \
./src/l2frame/con3g4g.d \
./src/l2frame/ethernet.d \
./src/l2frame/gps.d \
./src/l2frame/hwinv.d \
./src/l2frame/lcd.d \
./src/l2frame/modbus.d \
./src/l2frame/sps232.d \
./src/l2frame/sps485.d \
./src/l2frame/usbnet.d \
./src/l2frame/wifi.d 


# Each subdirectory must supply rules for building sources it contributes
src/l2frame/%.o: ../src/l2frame/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


