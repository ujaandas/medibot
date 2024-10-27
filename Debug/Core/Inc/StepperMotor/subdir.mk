################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Inc/StepperMotor/StepperMotor.cpp 

OBJS += \
./Core/Inc/StepperMotor/StepperMotor.o 

CPP_DEPS += \
./Core/Inc/StepperMotor/StepperMotor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/StepperMotor/%.o Core/Inc/StepperMotor/%.su: ../Core/Inc/StepperMotor/%.cpp Core/Inc/StepperMotor/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-StepperMotor

clean-Core-2f-Inc-2f-StepperMotor:
	-$(RM) ./Core/Inc/StepperMotor/StepperMotor.d ./Core/Inc/StepperMotor/StepperMotor.o ./Core/Inc/StepperMotor/StepperMotor.su

.PHONY: clean-Core-2f-Inc-2f-StepperMotor

