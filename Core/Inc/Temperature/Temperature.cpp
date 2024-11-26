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
    HAL_ADC_PollForConversion(hadc, 10);
    rawVal = HAL_ADC_GetValue(hadc);
}

uint32_t Temperature::getRawVal() const {
    return rawVal;
}

float Temperature::getTemperatureCelsius() const {
    float VREF = 3.3;
    return (rawVal * (VREF / 4096.0 * 100)) - 15; // Use reference voltage
}
