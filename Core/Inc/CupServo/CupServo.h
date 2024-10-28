/*
 * CupServo.h
 *
 *  Created on: Oct 28, 2024
 *      Author: ooj
 */

#ifndef __CUP_SERVO
#define __CUP_SERVO

#include "stm32f1xx_hal.h"
#include "ServoMotor/ServoMotor.h"

class CupServo: public ServoMotor {
	public:
		CupServo(uint16_t cupCount, TIM_HandleTypeDef* timer, uint16_t timerChannel);
		void selectCup(uint16_t cup);
	private:
		uint16_t cupCount;
};

#endif /* __CUP_SERVO */
