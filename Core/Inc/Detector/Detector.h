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
#define MAX_BASELINE_COLOURS 5

class Detector {
public:
    Detector(Camera& camera, uint16_t targetColours[], uint8_t colourCount, uint8_t threshold, uint16_t targetX, uint16_t targetY, uint16_t boxSize, void (*colourDetectedHandler)(uint16_t));

    void calibrate();
    void displayImage();
    uint16_t isColourDetected(uint16_t colour);

private:
    Camera& camera;
    uint16_t targetColours[MAX_TARGET_COLOURS];
    uint16_t baselineColours[MAX_BASELINE_COLOURS];
    uint8_t colourCount;
    uint8_t threshold;
    uint16_t targetX;
    uint16_t targetY;
    uint16_t boxSize;
    void (*colourDetectedHandler)(uint16_t);

    uint16_t calcAvgColour(uint32_t sumRed, uint32_t sumGreen, uint32_t sumBlue, uint32_t pixelCount);
    uint16_t getBackgroundColour(uint16_t color);
    int abs(int x);
};

#endif /* __DETECTOR_H_ */
