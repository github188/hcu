################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/l3app/hsmmp.c \
../src/l3app/sensoremc.c \
../src/l3app/sensorhumid.c \
../src/l3app/sensorpm25.c \
../src/l3app/sensortemp.c \
../src/l3app/sensorwinddir.c \
../src/l3app/sensorwindspd.c \
../src/l3app/svrcon.c 

OBJS += \
./src/l3app/hsmmp.o \
./src/l3app/sensoremc.o \
./src/l3app/sensorhumid.o \
./src/l3app/sensorpm25.o \
./src/l3app/sensortemp.o \
./src/l3app/sensorwinddir.o \
./src/l3app/sensorwindspd.o \
./src/l3app/svrcon.o 

C_DEPS += \
./src/l3app/hsmmp.d \
./src/l3app/sensoremc.d \
./src/l3app/sensorhumid.d \
./src/l3app/sensorpm25.d \
./src/l3app/sensortemp.d \
./src/l3app/sensorwinddir.d \
./src/l3app/sensorwindspd.d \
./src/l3app/svrcon.d 


# Each subdirectory must supply rules for building sources it contributes
src/l3app/%.o: ../src/l3app/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


