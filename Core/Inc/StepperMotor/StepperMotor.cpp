/*
 * Stepper.cpp
 *
 *  Created on: Oct 27, 2024
 *      Author: ooj
 */

#include <StepperMotor/StepperMotor.h>

StepperMotor::StepperMotor(GPIO_TypeDef* gpioPort, uint16_t pin1, uint16_t pin2, uint16_t pin3, uint16_t pin4, TIM_HandleTypeDef* timer)
    : gpioPort(gpioPort), pin1(pin1), pin2(pin2), pin3(pin3), pin4(pin4), timer(timer) {
	HAL_TIM_Base_Start(timer);
}

void StepperMotor::microDelay(uint16_t delay) {
    __HAL_TIM_SET_COUNTER(timer, 0);
    while (__HAL_TIM_GET_COUNTER(timer) < delay);
}

void StepperMotor::singleStep(int index) {
    HAL_GPIO_WritePin(gpioPort, pin1, state[index][0]);
    HAL_GPIO_WritePin(gpioPort, pin2, state[index][1]);
    HAL_GPIO_WritePin(gpioPort, pin3, state[index][2]);
    HAL_GPIO_WritePin(gpioPort, pin4, state[index][3]);
}

void StepperMotor::makeSteps(int steps, uint16_t delay, bool clockwise) {
    for (int x = 0; x < steps; ++x) {
        for (int i = 0; i < STEP_COUNT; ++i) {
            int index = clockwise ? (STEP_COUNT - 1 - i) : i;
            singleStep(index);
            microDelay(delay);
        }
    }
}
