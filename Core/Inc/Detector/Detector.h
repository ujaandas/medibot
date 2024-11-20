/*
 * Detector.h
 *
 *  Created on: Nov 20, 2024
 *      Author: ooj
 */

#ifndef __DETECTOR_H
#define __DETECTOR_H

#include "stm32f1xx_hal.h"
#include "Camera/Camera.h"

class Detector: public Camera {
	public:
		Detector(uint8_t size_x, uint8_t size_y, uint16_t colour);
		bool isDetected();

	private:

};

#endif /* __DETECTOR_H */
