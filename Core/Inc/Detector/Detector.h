/*
 * Detector.h
 *
 *  Created on: Nov 23, 2024
 *      Author: ooj
 */

#ifndef __DETECTOR_H_
#define __DETECTOR_H_

#include "Camera/Camera.h"
#include "Screen/lcd.h"
#include <stdio.h>

class Detector {
public:
    Detector(Camera& camera)
        : camera(camera) {}

    void displayImage(uint16_t targetX, uint16_t targetY, uint16_t boxSize);

private:
    Camera& camera;

    uint16_t calcAvgColour(uint32_t sumRed, uint32_t sumGreen, uint32_t sumBlue, uint32_t pixelCount);
};

#endif /* __DETECTOR_H_ */
