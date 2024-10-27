/*
 * Stepper.cpp
 *
 *  Created on: Oct 27, 2024
 *      Author: ooj
 */

#include "StepperMotor.h"

StepperMotor::StepperMotor(GPIO_TypeDef* GPIO_Port, uint16_t Pin1, uint16_t Pin2, uint16_t Pin3, uint16_t Pin4, TIM_HandleTypeDef* Timer)
    : GPIO_Port(GPIO_Port), Pin1(Pin1), Pin2(Pin2), Pin3(Pin3), Pin4(Pin4), Timer(Timer) {}

void StepperMotor::microDelay(uint16_t delay) {
    __HAL_TIM_SET_COUNTER(Timer, 0);
    while (__HAL_TIM_GET_COUNTER(Timer) < delay);
}

void StepperMotor::stepSingle(int index) {
    HAL_GPIO_WritePin(GPIO_Port, Pin1, state[index][0]);
    HAL_GPIO_WritePin(GPIO_Port, Pin2, state[index][1]);
    HAL_GPIO_WritePin(GPIO_Port, Pin3, state[index][2]);
    HAL_GPIO_WritePin(GPIO_Port, Pin4, state[index][3]);
}

void StepperMotor::stepMotor(int steps, uint16_t delay, bool clockwise) {
    for (int x = 0; x < steps; ++x) {
        for (int i = 0; i < STEP_COUNT; ++i) {
            int index = clockwise ? (STEP_COUNT - 1 - i) : i;
            stepSingle(index);
            microDelay(delay);
        }
    }
}
