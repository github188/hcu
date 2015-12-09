################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/comvm/configxml.c \
../src/comvm/mmc.c \
../src/comvm/sysinfo.c \
../src/comvm/timer.c \
../src/comvm/trace.c \
../src/comvm/vmlayer.c 

OBJS += \
./src/comvm/configxml.o \
./src/comvm/mmc.o \
./src/comvm/sysinfo.o \
./src/comvm/timer.o \
./src/comvm/trace.o \
./src/comvm/vmlayer.o 

C_DEPS += \
./src/comvm/configxml.d \
./src/comvm/mmc.d \
./src/comvm/sysinfo.d \
./src/comvm/timer.d \
./src/comvm/trace.d \
./src/comvm/vmlayer.d 


# Each subdirectory must supply rules for building sources it contributes
src/comvm/%.o: ../src/comvm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	/usr/local/arm/arm-2009q3/bin/arm-none-linux-gnueabi-gcc -DTARGET_LINUX_ARM -I/usr/local/arm/linux_arm_a8/include/libxml2 -I/usr/local/arm/linux_arm_a8_curl/include -O0 -lpthread -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


