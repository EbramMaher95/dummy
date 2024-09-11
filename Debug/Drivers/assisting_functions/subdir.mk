################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/assisting_functions/assisting_functions.c 

OBJS += \
./Drivers/assisting_functions/assisting_functions.o 

C_DEPS += \
./Drivers/assisting_functions/assisting_functions.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/assisting_functions/%.o Drivers/assisting_functions/%.su Drivers/assisting_functions/%.cyclo: ../Drivers/assisting_functions/%.c Drivers/assisting_functions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-assisting_functions

clean-Drivers-2f-assisting_functions:
	-$(RM) ./Drivers/assisting_functions/assisting_functions.cyclo ./Drivers/assisting_functions/assisting_functions.d ./Drivers/assisting_functions/assisting_functions.o ./Drivers/assisting_functions/assisting_functions.su

.PHONY: clean-Drivers-2f-assisting_functions

