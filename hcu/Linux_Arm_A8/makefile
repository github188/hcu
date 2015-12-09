################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/webser/subdir.mk
-include src/l3app/subdir.mk
-include src/l2frame/subdir.mk
-include src/comvm/subdir.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: hcu

# Tool invocations
hcu: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross GCC Linker'
	/usr/local/arm/arm-2009q3/bin/arm-none-linux-gnueabi-gcc -L/usr/local/arm/linux_arm_a8/lib -L/usr/local/arm/linux_arm_a8_curl/lib -o "hcu" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) hcu
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
