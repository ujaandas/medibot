################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/CupServo/CupServo.cpp 

OBJS += \
./Core/Inc/CupServo/CupServo.o 

CPP_DEPS += \
./Core/Inc/CupServo/CupServo.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/CupServo/%.o Core/Inc/CupServo/%.su: ../Core/Inc/CupServo/%.cpp Core/Inc/CupServo/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-CupServo

clean-Core-2f-Inc-2f-CupServo:
	-$(RM) ./Core/Inc/CupServo/CupServo.d ./Core/Inc/CupServo/CupServo.o ./Core/Inc/CupServo/CupServo.su

.PHONY: clean-Core-2f-Inc-2f-CupServo

