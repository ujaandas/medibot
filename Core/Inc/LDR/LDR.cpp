/*
 * LDR.cpp
 *
 *  Created on: Nov 4, 2024
 *      Author: ooj
 */

#include <LDR/LDR.h>
#include <stdio.h>

LDR::LDR(ADC_HandleTypeDef* hadc) : hadc(hadc) {
	HAL_ADCEx_Calibration_Start(hadc);
    HAL_ADC_Start(hadc);
}

void LDR::read() {
    HAL_ADC_PollForConversion(hadc, 1000);
    LDR::rawVal = HAL_ADC_GetValue(hadc);
    LDR::intensity = mapValue(rawVal);
}

uint8_t LDR::getIntensity() const {
    return LDR::intensity;
}

uint32_t LDR::getRawVal() const {
    return LDR::rawVal;
}

uint8_t LDR::mapValue(uint32_t adcVal) {
    if (adcVal <= 1700) return 0;
    if (adcVal >= 4096) return 100;
    return (adcVal - 1700) * 100 / (4096 - 1700);
}


