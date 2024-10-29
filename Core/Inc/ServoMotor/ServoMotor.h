/*
 * ServoMotor.h
 *
 *  Created on: Oct 27, 2024
 *      Author: ooj
 */

#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f1xx_hal.h"

class ServoMotor {
	public:
		ServoMotor(TIM_HandleTypeDef* timer, uint16_t timerChannel);

		void spinTo(uint16_t angle);
		void addDeg(uint16_t sup_angle);
	private:
		uint16_t minPulse;
		uint16_t maxPulse;
		uint16_t totalSteps;
		TIM_HandleTypeDef* timer;
		uint16_t timerChannel;
};

#endif /* __SERVO_H */
