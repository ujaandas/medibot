/*
 * CupServo.cpp
 *
 *  Created on: Oct 28, 2024
 *      Author: ooj
 */

#include <CupServo/CupServo.h>

CupServo::CupServo(uint16_t cupCount, TIM_HandleTypeDef* timer, uint16_t timerChannel):
	ServoMotor(timer, timerChannel), cupCount(cupCount) {}

void CupServo::selectCup(uint16_t cup, uint16_t totalCups){
    if (cup < totalCups) {
        uint16_t angle = (360 / totalCups) * cup;
        ServoMotor::spinTo(angle);
    }
}

