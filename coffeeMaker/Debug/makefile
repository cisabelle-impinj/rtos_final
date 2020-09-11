################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
coffeeMaker.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
coffeeMaker.list \

OBJCOPY_BIN += \
coffeeMaker.bin \


# All Target
all: coffeeMaker.elf secondary-outputs

# Tool invocations
coffeeMaker.elf: $(OBJS) $(USER_OBJS) G:\ucsdExtension\embeddedRTOS\final\coffeeMaker\STM32L475VGTX_FLASH.ld
	arm-none-eabi-gcc -o "coffeeMaker.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"G:\ucsdExtension\embeddedRTOS\final\coffeeMaker\STM32L475VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="coffeeMaker.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

coffeeMaker.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "coffeeMaker.list"
	@echo 'Finished building: $@'
	@echo ' '

coffeeMaker.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "coffeeMaker.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
