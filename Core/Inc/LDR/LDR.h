/*
 * LDR.h
 *
 *  Created on: Nov 4, 2024
 *      Author: ooj
 */

#ifndef __LDR
#define __LDR

#define MIN_VAL 1700
#define MAX_VAL 4096

#include "main.h"

class LDR {
	public:
		LDR(ADC_HandleTypeDef* hadc);
		void read();
		uint8_t getIntensity() const;
		uint32_t getRawVal() const;
		bool somethingPassed(uint8_t threshold);
		bool somethingBlocking(uint8_t threshold, uint32_t durationMs);

	private:
		ADC_HandleTypeDef* hadc;
		uint32_t rawVal;
		uint8_t intensity;
		uint8_t baselineIntensity;
		uint32_t lastCheckTimestamp;
		bool passingState;
		uint8_t mapValue(uint32_t adcVal);
};

#endif /* __LDR */
