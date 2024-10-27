/*
 * stepper.h
 *
 *  Created on: Oct 27, 2024
 *      Author: ooj
 */

#ifndef __STEPPER_H
#define __STEPPER_H

#include "stm32f1xx_hal.h"

#define STEP_COUNT 8
#define COIL_COUNT 4

// Stepper motor configuration struct
typedef struct {
    GPIO_TypeDef *GPIO_Port;
    uint16_t Pin1;
    uint16_t Pin2;
    uint16_t Pin3;
    uint16_t Pin4;
    TIM_HandleTypeDef *Timer;
} StepperMotor;

// Function prototypes
void microdelay(TIM_HandleTypeDef *Timer, uint16_t delay);
void step_single(StepperMotor *motor, int index);
void step_motor(StepperMotor *motor, int steps, uint16_t delay, int clockwise);



#endif /* __STEPPER_H */
