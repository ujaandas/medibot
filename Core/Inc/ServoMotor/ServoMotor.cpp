/*
 * ServoMotor.cpp
 *
 *  Created on: Oct 27, 2024
 *      Author: ooj
 */

#include <ServoMotor/ServoMotor.h>

ServoMotor::ServoMotor(TIM_HandleTypeDef* timer, uint16_t timerChannel) :
	timer(timer), timerChannel(timerChannel){
	HAL_TIM_PWM_Start(timer, timerChannel);
	uint16_t arr = timer->Init.Period;
	minPulse = arr * 0.025;
	maxPulse = arr * 0.125;
	totalSteps = maxPulse - minPulse;
}


void ServoMotor::spinTo(uint16_t angle) {
    if (angle > 180) angle = 180;
    if (angle < 0) angle = 0;

    uint16_t pulseWidth = minPulse + ((angle * totalSteps) / 180);

    __HAL_TIM_SET_COMPARE(timer, timerChannel, pulseWidth);
}


void ServoMotor::addDeg(uint16_t sup_angle){
}
