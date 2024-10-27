################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/Screen/lcd.cpp 

OBJS += \
./Core/Inc/Screen/lcd.o 

CPP_DEPS += \
./Core/Inc/Screen/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Screen/%.o Core/Inc/Screen/%.su: ../Core/Inc/Screen/%.cpp Core/Inc/Screen/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-Screen

clean-Core-2f-Inc-2f-Screen:
	-$(RM) ./Core/Inc/Screen/lcd.d ./Core/Inc/Screen/lcd.o ./Core/Inc/Screen/lcd.su

.PHONY: clean-Core-2f-Inc-2f-Screen

