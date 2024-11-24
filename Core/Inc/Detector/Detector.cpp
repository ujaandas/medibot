/*
 * Detector.cpp
 *
 *  Created on: Nov 23, 2024
 *      Author: ooj
 */

#include "Detector.h"

void Detector::displayImage(uint16_t targetX, uint16_t targetY, uint16_t boxSize) {
    if (!camera.isInitialized()) return;

    uint16_t halfBoxSize = boxSize / 2;
    uint32_t sumRed = 0, sumGreen = 0, sumBlue = 0, pixelCount = 0;

    camera.fifoController.prepareFIFO();
    LCD_Cam_Gram();

    for (uint16_t x = 0; x < 240; x++) {
        for (uint16_t y = 0; y < 320; y++) {
            uint16_t Camera_Data = camera.fifoController.readPixel();

            // Draw border
            bool onBorder =
                ((x == targetY - halfBoxSize || x == targetY + halfBoxSize)
                		&& (y >= targetX - halfBoxSize && y <= targetX + halfBoxSize)) ||
                ((y == targetX - halfBoxSize || y == targetX + halfBoxSize)
                		&& (x >= targetY - halfBoxSize && x <= targetY + halfBoxSize));

            if (onBorder) {
                LCD_Write_Data(RED);
            } else {
                LCD_Write_Data(Camera_Data);
            }

            // Bounded by box
            if (x >= targetY - halfBoxSize && x <= targetY + halfBoxSize &&
            		y >= targetX - halfBoxSize && y <= targetX + halfBoxSize) {
            	// rgb is 16-bit in 565 format (5b red, 6b green, 5b blue)
                uint16_t red = (Camera_Data & 0xF800) >> 11; // 1111100000000000
                uint16_t green = (Camera_Data & 0x07E0) >> 5; // 0000011111100000
                uint16_t blue = Camera_Data & 0x001F; // 0000000000011111

                sumRed += red << 3; // mult 2^3
                sumGreen += green << 2; // mult2^2
                sumBlue += blue << 3; // mult 2^3
                pixelCount++;
            }
        }
    }

    uint16_t averageColor = calcAvgColour(sumRed, sumGreen, sumBlue, pixelCount);
    char message[30];
    sprintf(message, "Avg Color: 0x%04X", averageColor);
    LCD_DrawString(50, 220, (uint8_t*)message);

    HAL_Delay(1000);
}

uint16_t Detector::calcAvgColour(uint32_t sumRed, uint32_t sumGreen, uint32_t sumBlue, uint32_t pixelCount) {
    if (pixelCount == 0) return 0;

    uint16_t avgRed = (sumRed / pixelCount) >> 3;
    uint16_t avgGreen = (sumGreen / pixelCount) >> 2;
    uint16_t avgBlue = (sumBlue / pixelCount) >> 3;

    return (avgRed << 11) | (avgGreen << 5) | avgBlue;
}

