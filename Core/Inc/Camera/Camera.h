/*
 * Camera.h
 *
 *  Created on: 19 Nov 2024
 *      Author: ooj
 */

#ifndef __CAMERA_H
#define __CAMERA_H

#include "stm32f1xx_hal.h"
#include "SCCBController/SCCBController.h"
#include "FIFOController/FIFOController.h"
#include "Screen/lcd.h"
#include "CameraRegisters.h"

class Camera {
	public:
		Camera(const SCCBController& sccbController, const FIFOController& fifoController);
		volatile uint8_t vsync;

		void init();
		bool isInitialized();
		void displayImage();
		void vsyncHandler();
		bool isColorAtPoint(uint16_t x, uint16_t y, uint16_t targetColor);

		friend class Detector;

	private:
		bool initialized;
		SCCBController sccbController;
		FIFOController fifoController;

		bool writeSensorReg(uint8_t addr, uint8_t data);
		bool readSensorReg(uint8_t addr, uint8_t& data);
		bool verifySensorID();
};

#endif // __CAMERA_H
