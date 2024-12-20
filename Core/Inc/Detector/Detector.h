/*
 * Detector.h
 *
 *  Created on: Nov 23, 2024
 *      Author: ooj
 */

#ifndef __DETECTOR_H_
#define __DETECTOR_H_

#include "Camera/Camera.h"
#include "ServoMotor/ServoMotor.h"
#include "Screen/lcd.h"
#include <stdio.h>
#include <math.h>

#define MAX_TARGET_COLOURS 10
#define MAX_BASELINE_COLOURS 5

struct RGB
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;

    RGB(uint16_t color)
    {
        red = (color & 0xF800) >> 11;  // Extract red (5 bits)
        green = (color & 0x07E0) >> 5; // Extract green (6 bits)
        blue = color & 0x001F;         // Extract blue (5 bits)
    }

    uint16_t toColor()
    {
        return (red << 11) | (green << 5) | blue;
    }
};

class Detector
{
public:
    Detector(Camera &camera, ServoMotor &armServo, uint16_t targetColours[], uint8_t desiredCount[], uint8_t colourCount, void (*colourDetectedHandler)(int32_t));

    void calibrate(uint16_t targetX, uint16_t targetY, uint16_t boxSize);
    bool isCalibrated();
    void displayImage(uint16_t targetX, uint16_t targetY, uint16_t boxSize);

    bool done = false;

private:
    Camera &camera;
    ServoMotor &armServo;
    uint16_t targetColours[MAX_TARGET_COLOURS];
    uint16_t prevTarget;
    uint8_t targetCount[MAX_TARGET_COLOURS];
    uint8_t desiredCount[MAX_TARGET_COLOURS];
    uint16_t baselineColours[MAX_BASELINE_COLOURS];
    uint16_t startAngle = 170;
    uint16_t currAngle = startAngle;
    uint16_t minAngle = 160;
    uint16_t TSLT = 0;
    uint8_t colourCount;
    //    uint16_t targetX;
    //    uint16_t targetY;
    //    uint16_t boxSize;
    void (*colourDetectedHandler)(int32_t);
    uint16_t calcAvgColour(uint32_t sumRed, uint32_t sumGreen, uint32_t sumBlue, uint32_t pixelCount);
    float getWeightedDistance(const RGB &color1, const RGB &color2);

    // Constants for color comparison
    static constexpr float RED_WEIGHT = 0.3f;
    static constexpr float GREEN_WEIGHT = 0.5f;
    static constexpr float BLUE_WEIGHT = 0.2f;

    // Constants for weighted threshold
    static constexpr float TARGET_THRESHOLD = 0.8f;
    static constexpr float BASELINE_THRESHOLD = 1.1f;
};

#endif /* __DETECTOR_H_ */
