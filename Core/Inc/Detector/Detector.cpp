/*
 * Detector.cpp
 *
 *  Created on: Nov 23, 2024
 *      Author: ooj
 */

#include "Detector.h"

Detector::Detector(Camera& camera, uint8_t colourCount, uint16_t targetColours[], uint8_t threshold, void (*colourDetectedHandler)(uint16_t))
	: camera(camera), colourCount(colourCount), threshold(threshold), colourDetectedHandler(colourDetectedHandler) {
	for (uint8_t i = 0; i < colourCount && i < MAX_TARGET_COLOURS; i++) {
		this->targetColours[i] = targetColours[i];
	}
}

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

    uint16_t averageColour = calcAvgColour(sumRed, sumGreen, sumBlue, pixelCount);
    averageColour = isColourDetected(averageColour);

    char message[30];
    sprintf(message, "Avg Color: 0x%04X", averageColour);
    LCD_DrawStringColor(50, 220, message, averageColour, 0xFFFF);

//    HAL_Delay(1000);
}

uint16_t Detector::calcAvgColour(uint32_t sumRed, uint32_t sumGreen, uint32_t sumBlue, uint32_t pixelCount) {
    if (pixelCount == 0) return 0;

    uint16_t avgRed = (sumRed / pixelCount) >> 3;
    uint16_t avgGreen = (sumGreen / pixelCount) >> 2;
    uint16_t avgBlue = (sumBlue / pixelCount) >> 3;

    return (avgRed << 11) | (avgGreen << 5) | avgBlue;
}

int Detector::abs(int x) {
	return (x > 0) ? x : -x;
}

uint16_t Detector::isColourDetected(uint16_t colour) {
    uint16_t red1 = (colour & 0xF800) >> 11;       // 5 bits (0-31)
    uint16_t green1 = (colour & 0x07E0) >> 5;      // 6 bits (0-63)
    uint16_t blue1 = colour & 0x001F;              // 5 bits (0-31)

    // Weighted thresholds based on channel bit depth
    const uint8_t redThreshold = threshold;         // For 5 bits
    const uint8_t greenThreshold = threshold * 2;   // For 6 bits
    const uint8_t blueThreshold = threshold;        // For 5 bits

    // Color similarity percentage (0-100)
    const uint8_t requiredSimilarity = 80;

    for (uint8_t i = 0; i <= colourCount; i++) {
        uint16_t targetColour = targetColours[i];
        uint16_t red2 = (targetColour & 0xF800) >> 11;
        uint16_t green2 = (targetColour & 0x07E0) >> 5;
        uint16_t blue2 = targetColour & 0x001F;

        // Calculate color similarity percentage
        uint8_t redSimilarity = 100 - ((abs(red1 - red2) * 100) / 31);
        uint8_t greenSimilarity = 100 - ((abs(green1 - green2) * 100) / 63);
        uint8_t blueSimilarity = 100 - ((abs(blue1 - blue2) * 100) / 31);

        uint8_t totalSimilarity = (redSimilarity + greenSimilarity + blueSimilarity) / 3;

        // Check if color is within threshold and meets similarity requirement
        if (totalSimilarity >= requiredSimilarity &&
            abs(red1 - red2) <= redThreshold &&
            abs(green1 - green2) <= greenThreshold &&
            abs(blue1 - blue2) <= blueThreshold) {

            colourDetectedHandler(targetColour);
            return targetColour;
        }
    }
    return colour;
}

