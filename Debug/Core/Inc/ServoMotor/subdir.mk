################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/ServoMotor/ServoMotor.cpp 

OBJS += \
./Core/Inc/ServoMotor/ServoMotor.o 

CPP_DEPS += \
./Core/Inc/ServoMotor/ServoMotor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/ServoMotor/%.o Core/Inc/ServoMotor/%.su: ../Core/Inc/ServoMotor/%.cpp Core/Inc/ServoMotor/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-ServoMotor

clean-Core-2f-Inc-2f-ServoMotor:
	-$(RM) ./Core/Inc/ServoMotor/ServoMotor.d ./Core/Inc/ServoMotor/ServoMotor.o ./Core/Inc/ServoMotor/ServoMotor.su

.PHONY: clean-Core-2f-Inc-2f-ServoMotor

