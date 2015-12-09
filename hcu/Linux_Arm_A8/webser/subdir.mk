################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../webser/get_time.c \
../webser/http_session.c \
../webser/init_socket.c \
../webser/webserver.c 

OBJS += \
./webser/get_time.o \
./webser/http_session.o \
./webser/init_socket.o \
./webser/webserver.o 

C_DEPS += \
./webser/get_time.d \
./webser/http_session.d \
./webser/init_socket.d \
./webser/webserver.d 


# Each subdirectory must supply rules for building sources it contributes
webser/%.o: ../webser/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	/usr/local/arm/arm-2009q3/bin/arm-none-linux-gnueabi-gcc -DTARGET_LINUX_ARM -I/usr/local/arm/linux_arm_a8/include/libxml2 -O0 -lpthread -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


