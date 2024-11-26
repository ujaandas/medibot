/*
 * Temperature.h
 *
 *  Created on: Nov 4, 2024
 *      Author: ooj
 */

#ifndef __Temperature
#define __Temperature

#define MIN_VAL 1700
#define MAX_VAL 4096

#include "main.h"

class Temperature {
	public:
		Temperature(ADC_HandleTypeDef* hadc);
		void read();
		uint32_t getRawVal() const;
		float getTemperatureCelsius() const;

	private:
		ADC_HandleTypeDef* hadc;
		uint32_t rawVal;
};

#endif /* __Temperature */
