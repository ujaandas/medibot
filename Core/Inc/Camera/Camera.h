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

		bool init();
		void displayImage();
		void vsyncHandler();

	private:
		bool initialized;
		SCCBController sccbController;
		FIFOController fifoController;

		bool writeSensorReg(uint8_t addr, uint8_t data);
		bool readSensorReg(uint8_t addr, uint8_t& data);
		bool verifySensorID();
};

#endif // __CAMERA_H
