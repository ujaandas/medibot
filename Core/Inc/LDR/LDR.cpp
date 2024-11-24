/*
 * LDR.cpp
 *
 *  Created on: Nov 4, 2024
 *      Author: ooj
 */

#include <LDR/LDR.h>
#include <stdio.h>

LDR::LDR(ADC_HandleTypeDef* hadc) :
	hadc(hadc), rawVal(0), intensity(0), baselineIntensity(0) {
	HAL_ADCEx_Calibration_Start(hadc);
    HAL_ADC_Start(hadc);
    read();
    baselineIntensity = intensity;
}

void LDR::read() {
    HAL_ADC_PollForConversion(hadc, 1000);
    rawVal = HAL_ADC_GetValue(hadc);
    intensity = mapValue(rawVal);
}

uint8_t LDR::getIntensity() const {
    return intensity;
}

uint32_t LDR::getRawVal() const {
    return rawVal;
}

uint8_t LDR::mapValue(uint32_t adcVal) {
    if (adcVal <= MIN_VAL) return 0;
    if (adcVal >= MAX_VAL) return 100;
    return (adcVal - MIN_VAL) * 100 / (MAX_VAL - MIN_VAL);
}

bool LDR::somethingPassed(uint8_t threshold) {
	return (intensity < baselineIntensity - threshold);
}

bool LDR::somethingBlocking(uint8_t threshold, uint32_t durationMs) {
	uint32_t currentTime = HAL_GetTick();
	if (somethingPassed(threshold)) {
		if (!passingState) {
			passingState = true;
			lastCheckTimestamp = currentTime;
		}
		else if ((currentTime - lastCheckTimestamp) >= durationMs) {
			return true;
		}
	}
	else {
		passingState = false;
	}
	return false;
}


