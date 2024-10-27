/*
 * stepper.c
 *
 *  Created on: Oct 27, 2024
 *      Author: ooj
 */
#include "stepper.h"

const GPIO_PinState state[STEP_COUNT][COIL_COUNT] = {
    {GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET},
    {GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET},
    {GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET},
    {GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET},
    {GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET},
    {GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_SET},
    {GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET},
    {GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET}
};

void microDelay(TIM_HandleTypeDef *Timer, uint16_t delay) {
    __HAL_TIM_SET_COUNTER(Timer, 0);
    while (__HAL_TIM_GET_COUNTER(Timer) < delay);
}

void step_single(StepperMotor *motor, int index) {
    HAL_GPIO_WritePin(motor->GPIO_Port, motor->Pin1, state[index][0]);
    HAL_GPIO_WritePin(motor->GPIO_Port, motor->Pin2, state[index][1]);
    HAL_GPIO_WritePin(motor->GPIO_Port, motor->Pin3, state[index][2]);
    HAL_GPIO_WritePin(motor->GPIO_Port, motor->Pin4, state[index][3]);
}


void step_motor(StepperMotor *motor, int steps, uint16_t delay, int clockwise) {
    for (int x = 0; x < steps; ++x) {
        for (int i = 0; i < STEP_COUNT; ++i) {
            int index = clockwise ? (STEP_COUNT - 1) - i : i;
            step_single(motor, index);
            microDelay(motor->Timer, delay);
        }
    }
}

