################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Device_Drivers/24C32_EEPROM/24C32.c 

OBJS += \
./Drivers/Device_Drivers/24C32_EEPROM/24C32.o 

C_DEPS += \
./Drivers/Device_Drivers/24C32_EEPROM/24C32.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Device_Drivers/24C32_EEPROM/%.o Drivers/Device_Drivers/24C32_EEPROM/%.su Drivers/Device_Drivers/24C32_EEPROM/%.cyclo: ../Drivers/Device_Drivers/24C32_EEPROM/%.c Drivers/Device_Drivers/24C32_EEPROM/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Device_Drivers-2f-24C32_EEPROM

clean-Drivers-2f-Device_Drivers-2f-24C32_EEPROM:
	-$(RM) ./Drivers/Device_Drivers/24C32_EEPROM/24C32.cyclo ./Drivers/Device_Drivers/24C32_EEPROM/24C32.d ./Drivers/Device_Drivers/24C32_EEPROM/24C32.o ./Drivers/Device_Drivers/24C32_EEPROM/24C32.su

.PHONY: clean-Drivers-2f-Device_Drivers-2f-24C32_EEPROM

