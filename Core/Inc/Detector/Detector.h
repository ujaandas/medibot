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

#define MAX_TARGET_COLOURS 10

class Detector {
public:
    Detector(Camera& camera, uint8_t colourCount, uint16_t targetColours[], uint8_t threshold, void (*colourDetectedHandler)(uint16_t));

    void displayImage(uint16_t targetX, uint16_t targetY, uint16_t boxSize);
    uint16_t isColourDetected(uint16_t colour);

private:
    Camera& camera;
    uint16_t targetColours[MAX_TARGET_COLOURS];
    uint8_t colourCount;
    uint8_t threshold;
    void (*colourDetectedHandler)(uint16_t);

    uint16_t calcAvgColour(uint32_t sumRed, uint32_t sumGreen, uint32_t sumBlue, uint32_t pixelCount);
    uint16_t getBackgroundColour(uint16_t color);
    int abs(int x);
};

#endif /* __DETECTOR_H_ */
