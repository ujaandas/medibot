/*
 * Temperature.cpp
 *
 *  Created on: Nov 4, 2024
 *      Author: ooj
 */

#include <Temperature/Temperature.h>
#include <stdio.h>

Temperature::Temperature(ADC_HandleTypeDef* hadc) :
	hadc(hadc), rawVal(0) {
	HAL_ADCEx_Calibration_Start(hadc);
    HAL_ADC_Start(hadc);
    read();
}

void Temperature::read() {
    HAL_ADC_PollForConversion(hadc, 1000);
    rawVal = HAL_ADC_GetValue(hadc);
}

uint32_t Temperature::getRawVal() const {
    return rawVal;
}

uint8_t Temperature::mapValue(uint32_t adcVal) {
    if (adcVal <= MIN_VAL) return 0;
    if (adcVal >= MAX_VAL) return 100;
    return (adcVal - MIN_VAL) * 100 / (MAX_VAL - MIN_VAL);
}
