/*
 * Detector.cpp
 *
 *  Created on: Nov 23, 2024
 *      Author: ooj
 */

#include "Detector.h"

Detector::Detector(Camera &camera, uint16_t targetColours[], uint8_t colourCount, uint8_t threshold, uint16_t targetX, uint16_t targetY, uint16_t boxSize, void (*colourDetectedHandler)(uint16_t))
    : camera(camera), colourCount(colourCount), threshold(threshold), targetX(targetX), targetY(targetY), boxSize(boxSize), colourDetectedHandler(colourDetectedHandler)
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

void Detector::calibrate()
{
    if (!camera.isInitialized())
        return;

    uint8_t baselineCount = 0;
    // Calibrate until baseline colours are set
    while (baselineCount < MAX_BASELINE_COLOURS)
    {
        LCD_DrawString(50, 220, (uint8_t *)"Calibrating...");
        uint16_t halfBoxSize = boxSize / 2;

        camera.fifoController.prepareFIFO();
        LCD_Cam_Gram();

        for (uint16_t x = 0; x < 240; x++)
        {
            for (uint16_t y = 0; y < 320; y++)
            {
                uint16_t Camera_Data = camera.fifoController.readPixel();

                // Bounded by box
                if (x >= targetY - halfBoxSize && x <= targetY + halfBoxSize &&
                    y >= targetX - halfBoxSize && y <= targetX + halfBoxSize)
                {
                    baselineColours[baselineCount++] = Camera_Data;
                    char message[30];
                    sprintf(message, "Calibrating 0x%04X...", Camera_Data);
                    LCD_DrawString(50, 220, (uint8_t *)message);
                }
            }
        }
        HAL_Delay(1000);
    }
}

void Detector::displayImage()
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

            if (onBorder) {
            	char borderMessage[30];
            	sprintf(borderMessage, "Border at (%d, %d)", x, y);
            	LCD_DrawString(0, 200, (uint8_t *)borderMessage);
            }

            // Bounded by box
            if (x >= targetY - halfBoxSize && x <= targetY + halfBoxSize &&
                y >= targetX - halfBoxSize && y <= targetX + halfBoxSize)
            {

            	char boxMessage[30];
            	sprintf(boxMessage, "Box at (%d, %d)", x, y);
            	LCD_DrawString(0, 220, (uint8_t *)boxMessage);

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
    char message[30];
    sprintf(message, "Pixel Count: %d", pixelCount);
    //    // uint16_t detectedColour = isColourDetected(averageColour);
    //
    //    // Check if detected colour is in baseline colours
    //    for (uint8_t i = 0; i < MAX_BASELINE_COLOURS; i++)
    //    {
    //        if (averageColour == baselineColours[i])
    //        {
    //            sprintf(message, "Baseline 0x%04X", averageColour);
    //            break;
    //        }
    //        else
    //        {
    //            // averageColour = isColourDetected(averageColour);
    //            sprintf(message, "Avg Color: 0x%04X", averageColour);
    //        }
    //    }

	LCD_DrawString(50, 220, (uint8_t *)message);
    HAL_Delay(1000);
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

int Detector::abs(int x)
{
    return (x > 0) ? x : -x;
}

uint16_t Detector::isColourDetected(uint16_t colour)
{
    uint16_t red1 = (colour & 0xF800) >> 11;
    uint16_t green1 = (colour & 0x07E0) >> 5;
    uint16_t blue1 = colour & 0x001F;

    for (uint8_t i = 0; i < colourCount; i++)
    {
        uint16_t targetColour = targetColours[i];
        uint16_t red2 = (targetColour & 0xF800) >> 11;
        uint16_t green2 = (targetColour & 0x07E0) >> 5;
        uint16_t blue2 = (targetColour & 0x001F);

        if ((abs(red1 - red2) <= threshold) &&
            (abs(green1 - green2) <= threshold) &&
            (abs(blue1 - blue2) <= threshold))
        {
            colourDetectedHandler(targetColour);
            return targetColour;
        }
    }
    return colour;
}
