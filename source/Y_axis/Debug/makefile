################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
Y_axis.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Y_axis.list \

OBJCOPY_BIN += \
Y_axis.bin \


# All Target
all: Y_axis.elf secondary-outputs

# Tool invocations
Y_axis.elf: $(OBJS) $(USER_OBJS) /Volumes/Macintosh\ HD/Utenti/Lorenzo/Documents/GitHub/ProStick64/source/Y_axis/STM32G031J6MX_FLASH.ld
	arm-none-eabi-gcc -o "Y_axis.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"/Volumes/Macintosh HD/Utenti/Lorenzo/Documents/GitHub/ProStick64/source/Y_axis/STM32G031J6MX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Y_axis.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Y_axis.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Y_axis.list"
	@echo 'Finished building: $@'
	@echo ' '

Y_axis.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Y_axis.bin"
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
