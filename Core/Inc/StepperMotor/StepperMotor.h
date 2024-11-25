/*
 * Stepper.h
 *
 *  Created on: Oct 27, 2024
 *      Author: ooj
 */

#ifndef __STEPPER_H
#define __STEPPER_H

#include "stm32f1xx_hal.h"
#include "GPIOPin.h"

#define STEP_COUNT 8
#define COIL_COUNT 4

class StepperMotor {
	public:
		StepperMotor(GPIOPin pin1, GPIOPin pin2, GPIOPin pin3, GPIOPin pin4, TIM_HandleTypeDef* timer);
		void makeSteps(int steps, uint16_t delay, bool clockwise);
	private:
	    GPIO_TypeDef* gpioPort;
	    GPIOPin  pin1;
	    GPIOPin  pin2;
	    GPIOPin  pin3;
	    GPIOPin  pin4;
	    TIM_HandleTypeDef* timer;

	    void microDelay(uint16_t delay);
	    void singleStep(int index);

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
};


#endif /* __STEPPER_H */
