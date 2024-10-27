/*
 * Stepper.h
 *
 *  Created on: Oct 27, 2024
 *      Author: ooj
 */

#ifndef __STEPPER_H
#define __STEPPER_H

#include "stm32f1xx_hal.h"

#define STEP_COUNT 8
#define COIL_COUNT 4

class StepperMotor {
	public:
		StepperMotor(GPIO_TypeDef* GPIO_Port, uint16_t Pin1, uint16_t Pin2, uint16_t Pin3, uint16_t Pin4, TIM_HandleTypeDef* Timer);
		void stepMotor(int steps, uint16_t delay, bool clockwise);
	private:
	    GPIO_TypeDef* GPIO_Port;
	    uint16_t Pin1;
	    uint16_t Pin2;
	    uint16_t Pin3;
	    uint16_t Pin4;
	    TIM_HandleTypeDef* Timer;

	    void microDelay(uint16_t delay);
	    void stepSingle(int index);

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
