/*
 * Camera.h
 *
 *  Created on: 19 Nov 2024
 *      Author: ooj
 */

#ifndef __CAMERA_H
#define __CAMERA_H

#include "stm32f1xx_hal.h"
#include "SCCB/SCCB.h"
#include "Helper/CameraRegisters.h"

class Camera {
	public:
		Camera(GPIO_TypeDef* sccbPort, uint16_t sclPin, uint16_t sdaPin);

		bool init();
		void displayImage();
		bool isInitialized() const { return initialized_; }

	private:
		bool initialized_;
		SCCB sccb_;

		bool writeSensorReg(uint8_t addr, uint8_t data);
		bool readSensorReg(uint8_t addr, uint8_t& data);
		bool verifySensorID();
};

#endif // __CAMERA_H
