/*
 * CupServo.cpp
 *
 *  Created on: Oct 28, 2024
 *      Author: ooj
 */

#include <CupServo/CupServo.h>

CupServo::CupServo(uint16_t totalCups, TIM_HandleTypeDef* timer, uint16_t timerChannel):
	ServoMotor(timer, timerChannel), totalCups(totalCups) {}

void CupServo::selectCup(uint16_t cup){
//    if (cup < totalCups) {
//        uint16_t angle = (360 / totalCups) * cup;
//        ServoMotor::spinTo(angle);
//    }
		if (cup == 0) {
			ServoMotor::spinTo(0);
		} else if (cup == 1) {
			ServoMotor::spinTo(90);
		} else if (cup == 2) {
			ServoMotor::spinTo(180);
		}
}

