################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/webser/get_time.c \
../src/webser/http_session.c \
../src/webser/init_socket.c \
../src/webser/webserver.c 

OBJS += \
./src/webser/get_time.o \
./src/webser/http_session.o \
./src/webser/init_socket.o \
./src/webser/webserver.o 

C_DEPS += \
./src/webser/get_time.d \
./src/webser/http_session.d \
./src/webser/init_socket.d \
./src/webser/webserver.d 


# Each subdirectory must supply rules for building sources it contributes
src/webser/%.o: ../src/webser/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	/usr/local/arm/arm-2009q3/bin/arm-none-linux-gnueabi-gcc -DTARGET_LINUX_ARM -I/usr/local/arm/linux_arm_a8/include/libxml2 -I/usr/local/arm/linux_arm_a8_curl/include -O0 -lpthread -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


