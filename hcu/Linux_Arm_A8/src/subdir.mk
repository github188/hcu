################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/hcu.c \
../src/hcuref.c 

OBJS += \
./src/hcu.o \
./src/hcuref.o 

C_DEPS += \
./src/hcu.d \
./src/hcuref.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	/usr/local/arm/arm-2009q3/bin/arm-none-linux-gnueabi-gcc -DTARGET_LINUX_ARM -I/usr/local/arm/linux_arm_a8/include/libxml2 -I/usr/local/arm/linux_arm_a8_curl/include -O0 -lpthread -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


