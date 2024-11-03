/*
 * LDR.h
 *
 *  Created on: Nov 4, 2024
 *      Author: ooj
 */

#ifndef __LDR
#define __LDR

#include "main.h"

class LDR {
	public:
		LDR(ADC_HandleTypeDef* hadc);
		void read();
		uint8_t getIntensity() const;
		uint32_t getRawVal() const;

	private:
		ADC_HandleTypeDef* hadc;
		uint32_t rawVal;
		uint8_t intensity;
		uint8_t mapValue(uint32_t adcVal);
};

#endif /* __LDR */
