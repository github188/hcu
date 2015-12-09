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
	gcc -DTARGET_LINUX_X86 -I/usr/include/libxml2 -I/usr/include/curl -O0 -lpthread -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


