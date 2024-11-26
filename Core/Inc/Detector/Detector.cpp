/*
 * Detector.cpp
 *
 *  Created on: Nov 23, 2024
 *      Author: ooj
 */

#include "Detector.h"

Detector::Detector(Camera &camera, uint16_t targetColours[], uint8_t colourCount, uint8_t threshold, void (*colourDetectedHandler)(uint16_t))
    : camera(camera), colourCount(colourCount), threshold(threshold), colourDetectedHandler(colourDetectedHandler)
{
    for (uint8_t i = 0; i < colourCount && i < MAX_TARGET_COLOURS; i++)
    {
        this->targetColours[i] = targetColours[i];
    }

    for (uint8_t i = 0; i < MAX_BASELINE_COLOURS; i++)
    {
        baselineColours[i] = 0;
    }
}

void Detector::calibrate(uint16_t targetX, uint16_t targetY, uint16_t boxSize)
{
    if (!camera.isInitialized())
        return;

    uint16_t halfBoxSize = boxSize / 2;
    uint32_t sumRed = 0, sumGreen = 0, sumBlue = 0, pixelCount = 0;

    camera.fifoController.prepareFIFO();
    LCD_Cam_Gram();

    for (uint16_t x = 0; x < 240; x++)
    {
        for (uint16_t y = 0; y < 320; y++)
        {
            uint16_t Camera_Data = camera.fifoController.readPixel();

            // Draw border
            bool onBorder =
                ((x == targetY - halfBoxSize || x == targetY + halfBoxSize) && (y >= targetX - halfBoxSize && y <= targetX + halfBoxSize)) ||
                ((y == targetX - halfBoxSize || y == targetX + halfBoxSize) && (x >= targetY - halfBoxSize && x <= targetY + halfBoxSize));

            onBorder ? LCD_Write_Data(RED) : LCD_Write_Data(Camera_Data);

            // Bounded by box
            if (x >= targetY - halfBoxSize && x <= targetY + halfBoxSize &&
                y >= targetX - halfBoxSize && y <= targetX + halfBoxSize)
            {
                // rgb is 16-bit in 565 format (5b red, 6b green, 5b blue)
                uint16_t red = (Camera_Data & 0xF800) >> 11;  // 1111100000000000
                uint16_t green = (Camera_Data & 0x07E0) >> 5; // 0000011111100000
                uint16_t blue = Camera_Data & 0x001F;         // 0000000000011111

                sumRed += red << 3;     // mult 2^3
                sumGreen += green << 2; // mult2^2
                sumBlue += blue << 3;   // mult 2^3
                pixelCount++;
            }
        }
    }

    uint16_t averageColour = calcAvgColour(sumRed, sumGreen, sumBlue, pixelCount);
    // uint16_t detectedColour = isColourDetected(averageColour);

    // Find next empty slot in baselineColours
    for (uint8_t i = 0; i < MAX_BASELINE_COLOURS; i++)
    {
        // Add average colour to baselineColours if slot is empty
        if (baselineColours[i] == 0)
        {
            baselineColours[i] = averageColour;
            char message[30];
            sprintf(message, "Baseline %d: 0x%04X", i, averageColour);
            LCD_DrawString(50, 220, (uint8_t *)message);
            break;
        }
    }
}

bool Detector::isCalibrated()
{
    for (uint8_t i = 0; i < MAX_BASELINE_COLOURS; i++)
    {
        if (baselineColours[i] == 0)
            return false;
    }
    return true;
}

void Detector::displayImage(uint16_t targetX, uint16_t targetY, uint16_t boxSize)
{
    if (!camera.isInitialized())
        return;

    uint16_t halfBoxSize = boxSize / 2;
    uint32_t sumRed = 0, sumGreen = 0, sumBlue = 0, pixelCount = 0;

    camera.fifoController.prepareFIFO();
    LCD_Cam_Gram();

    for (uint16_t x = 0; x < 240; x++)
    {
        for (uint16_t y = 0; y < 320; y++)
        {
            uint16_t Camera_Data = camera.fifoController.readPixel();

            // Draw border
            bool onBorder =
                ((x == targetY - halfBoxSize || x == targetY + halfBoxSize) && (y >= targetX - halfBoxSize && y <= targetX + halfBoxSize)) ||
                ((y == targetX - halfBoxSize || y == targetX + halfBoxSize) && (x >= targetY - halfBoxSize && x <= targetY + halfBoxSize));

            onBorder ? LCD_Write_Data(RED) : LCD_Write_Data(Camera_Data);

            // Bounded by box
            if (x >= targetY - halfBoxSize && x <= targetY + halfBoxSize &&
                y >= targetX - halfBoxSize && y <= targetX + halfBoxSize)
            {
                // rgb is 16-bit in 565 format (5b red, 6b green, 5b blue)
                uint16_t red = (Camera_Data & 0xF800) >> 11;  // 1111100000000000
                uint16_t green = (Camera_Data & 0x07E0) >> 5; // 0000011111100000
                uint16_t blue = Camera_Data & 0x001F;         // 0000000000011111

                sumRed += red << 3;     // mult 2^3
                sumGreen += green << 2; // mult2^2
                sumBlue += blue << 3;   // mult 2^3
                pixelCount++;
            }
        }
    }

    uint16_t averageColour = calcAvgColour(sumRed, sumGreen, sumBlue, pixelCount);

    // Get closest baseline colour
    uint16_t closestBaselineColour = 0;
    float minBaselineDist = 1000.0f;
    for (uint8_t i = 0; i < MAX_BASELINE_COLOURS; i++)
    {
        float distance = getWeightedDistance(RGB(averageColour), RGB(baselineColours[i])) * BASELINE_THRESHOLD;
        if (distance < minBaselineDist)
        {
            minBaselineDist = distance;
            closestBaselineColour = baselineColours[i];
        }
    }

    // Print closest baseline colour
    char message[30];
    sprintf(message, "Closest Baseline: 0x%04X", closestBaselineColour);
    LCD_DrawString(10, 10, (uint8_t *)message);

    // Print distance
    sprintf(message, "Dist to baseline: %.2f", minBaselineDist);
    LCD_DrawString(10, 30, (uint8_t *)message);

    // Get closest target colour
    uint16_t closestTargetColour = 0;
    float minTargetDist = 1000.0f;
    for (uint8_t i = 0; i < colourCount; i++)
    {
        float distance = getWeightedDistance(RGB(averageColour), RGB(targetColours[i])) * TARGET_THRESHOLD;
        if (distance < minTargetDist)
        {
            minTargetDist = distance;
            closestTargetColour = targetColours[i];
        }
    }
    // Print closest target colour
    sprintf(message, "Closest Target: 0x%04X", closestTargetColour);
    LCD_DrawString(10, 50, (uint8_t *)message);

    // Print distance
    sprintf(message, "Dist to target: %.2f", minTargetDist);
    LCD_DrawString(10, 70, (uint8_t *)message);

    // Print average colour
    sprintf(message, "Average: 0x%04X", averageColour);
    LCD_DrawString(10, 90, (uint8_t *)message);

    // See which distance is smaller
    if (minBaselineDist < minTargetDist)
    {
        // Print closest baseline colour
        sprintf(message, "Baseline: 0x%04X", closestBaselineColour);
        LCD_DrawString(50, 220, (uint8_t *)message);
    }
    else
    {
        // Print closest target colour
        sprintf(message, "Target: 0x%04X", closestTargetColour);
        LCD_DrawString(50, 220, (uint8_t *)message);
        colourDetectedHandler(closestTargetColour);
    }

    HAL_Delay(5000);
}

float Detector::getWeightedDistance(const RGB &color1, const RGB &color2)
{
    float redDiff = abs(static_cast<float>(color1.red) - color2.red);
    float greenDiff = abs(static_cast<float>(color1.green) - color2.green);
    float blueDiff = abs(static_cast<float>(color1.blue) - color2.blue);

    float distance = sqrt(RED_WEIGHT * redDiff * redDiff + GREEN_WEIGHT * greenDiff * greenDiff + BLUE_WEIGHT * blueDiff * blueDiff);

    return distance;
}

uint16_t Detector::calcAvgColour(uint32_t sumRed, uint32_t sumGreen, uint32_t sumBlue, uint32_t pixelCount)
{
    if (pixelCount == 0)
        return 0;

    uint16_t avgRed = (sumRed / pixelCount) >> 3;
    uint16_t avgGreen = (sumGreen / pixelCount) >> 2;
    uint16_t avgBlue = (sumBlue / pixelCount) >> 3;

    return (avgRed << 11) | (avgGreen << 5) | avgBlue;
}
