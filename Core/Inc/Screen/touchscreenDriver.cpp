#include <Screen/lcd.h>
#include <stdio.h>
#include <string.h>
#include <Screen/touchscreenDriver.h>

void DelayMicroseconds(__IO uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        for (volatile uint32_t j = 0; j < 12; j++) {
            // Busy wait
        }
    }
}

void WriteCommand(uint8_t command) {
    TOUCHSCREEN_MOSI_CLEAR();
    TOUCHSCREEN_CLK_LOW();

    for (int i = 0; i < 8; i++) {
        if (command & (1 << (7 - i))) {
            TOUCHSCREEN_MOSI_SET();
        } else {
            TOUCHSCREEN_MOSI_CLEAR();
        }
        DelayMicroseconds(5);
        TOUCHSCREEN_CLK_HIGH();
        DelayMicroseconds(5);
        TOUCHSCREEN_CLK_LOW();
    }
}

uint16_t ReadCommand(void) {
    uint16_t buffer = 0;
    TOUCHSCREEN_MOSI_CLEAR();
    TOUCHSCREEN_CLK_HIGH();

    for (int i = 0; i < 12; i++) {
        TOUCHSCREEN_CLK_LOW();
        uint8_t temp = TOUCHSCREEN_MISO_READ();
        buffer |= (temp << (11 - i));
        TOUCHSCREEN_CLK_HIGH();
    }

    return buffer;
}

uint16_t ReadAdcValue(uint8_t channel) {
    WriteCommand(channel);
    return ReadCommand();
}

void ReadAdcCoordinates(int16_t *xReading, int16_t *yReading) {
    int16_t xValue = ReadAdcValue(TOUCHSCREEN_CHANNEL_X);
    DelayMicroseconds(1);
    int16_t yValue = ReadAdcValue(TOUCHSCREEN_CHANNEL_Y);
    *xReading = xValue;
    *yReading = yValue;
}

Coordinate ConvertRawToCoordinates(int rawX, int rawY) {
    Coordinate adjustedCoords;
    Coordinate finalCoords;

    adjustedCoords.x = (rawY - 170 > 10000) ? 0 : (rawY - 170);
    adjustedCoords.y = (3740 - rawX > 10000) ? 0 : (3740 - rawX);

    finalCoords.x = (240.0f / 3700) * adjustedCoords.x;
    finalCoords.y = (320.0f / 3500) * adjustedCoords.y;

    return finalCoords;
}

uint8_t ReadSmoothedCoordinates(Coordinate *screenCoordinate) {
    uint8_t sampleCount = 0;
    int16_t sampleBuffer[2][10] = {0};
    int16_t adcValueX, adcValueY;

    while ((TOUCHSCREEN_EXTI_READ() == TOUCHSCREEN_EXTI_ACTIVE_LEVEL) && (sampleCount < 10)) {
        ReadAdcCoordinates(&adcValueX, &adcValueY);
        sampleBuffer[0][sampleCount] = adcValueX;
        sampleBuffer[1][sampleCount] = adcValueY;
        sampleCount++;
    }

    if (TOUCHSCREEN_EXTI_READ() != TOUCHSCREEN_EXTI_ACTIVE_LEVEL) {
        isScreenTouched = 0;
    }

    if (sampleCount == 10) {
        int32_t sumX = 0, sumY = 0;
        for (uint8_t i = 0; i < 10; i++) {
            sumX += sampleBuffer[0][i];
            sumY += sampleBuffer[1][i];
        }

        // Calculate average raw coordinates
        int16_t avgRawX = (sumX / 10);
        int16_t avgRawY = (sumY / 10);

        // Convert raw coordinates to scaled coordinates
        *screenCoordinate = ConvertRawToCoordinates(avgRawX, avgRawY);
        return 1; // Success
    }

    return 0; // Not enough samples
}
void ToggleLED(void) {
    Coordinate coords;
    while (1) {
        if (ReadSmoothedCoordinates(&coords)) {
            Coordinate finalCoords = ConvertRawToCoordinates(coords.x, coords.y);

            if ((finalCoords.y > 232) && (finalCoords.y < 282) &&
                (finalCoords.x > 95) && (finalCoords.x < 145)) {
            	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
            }

            char buffer[50];
            snprintf(buffer, sizeof(buffer), "Coords: (%d, %d)", finalCoords.x, finalCoords.y);
            LCD_DrawString(10, 10, (uint8_t*) buffer);
            break;
        }
    }
}

void CycleLedGradient(uint32_t durationMs) {
    const uint32_t delayPerStep = (durationMs * 1000) / 3; // Convert to microseconds

    // Transition from red to green
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // Red off
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Green on
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Blue off
    DelayMicroseconds(delayPerStep); // Wait for the duration of the step

    // Transition from green to blue
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // Red off
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Green off
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // Blue on
    DelayMicroseconds(delayPerStep); // Wait for the duration of the step

    // Transition from blue to red
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // Red on
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Green off
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Blue off
    DelayMicroseconds(delayPerStep); // Wait for the duration of the step

    // Turn off the LED after the cycle is complete
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // Red off
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Green off
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Blue off
}

void DisplayPatientSelection(PatientDetails *patients, uint8_t numPatients) {
		uint16_t startY = 0; // Starting Y position
	    uint16_t blockColor;

	    LCD_Clear(0,0,239,319,WHITE);

	    // Title with black background
	    LCD_Clear(0, startY, DISPLAY_WIDTH, BLOCK_HEIGHT, BLACK);
	    LCD_DrawStringColor(60, startY + 20, "Select a patient", WHITE, BLACK);
	    startY += BLOCK_HEIGHT + GAP;

	    // Buttons
	    for (uint16_t i = 0; i < numPatients; i++) {
	        blockColor = BLUE; //(i % 2 == 0) ? BLUE : GREEN;

	        LCD_Clear(GAP, startY, DISPLAY_WIDTH - (2*GAP), BLOCK_HEIGHT, blockColor);
	        LCD_DrawStringColor(30, startY + 20, patients[i].name, WHITE, blockColor);

	        startY += BLOCK_HEIGHT + GAP;
	    }
}

void DisplayPatientOptions(PatientDetails *patient) {
	uint16_t startY = 0; // Starting Y position
    uint16_t blockColor;

    // Title with black background
    LCD_Clear(0, startY, DISPLAY_WIDTH, BLOCK_HEIGHT, BLACK);
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "How can I help %s?", patient->name);
    LCD_DrawStringColor(35, startY + 20, buffer, WHITE, BLACK);
    startY += BLOCK_HEIGHT + GAP;

    const char *options[] = {
        "Take temperature",
        "Dispense medication"
    };

    // Buttons
    for (uint16_t i = 0; i < 2; i++) {
        blockColor = MAGENTA; //(i % 2 == 0) ? BLUE : GREEN;

        LCD_Clear(GAP, startY, DISPLAY_WIDTH - (2*GAP), BLOCK_HEIGHT, blockColor);
        LCD_DrawStringColor(30, startY + 20, options[i], WHITE, blockColor);

        startY += BLOCK_HEIGHT + GAP;
    }

    // Clear the rest of the screen to remove the previous buttons
    LCD_Clear(0,3*(BLOCK_HEIGHT + GAP),239,319,WHITE);
}

void DisplayTakingVitals(PatientDetails *patient) {
	uint16_t startY = 0; // Starting Y position
    LCD_Clear(0,0,239,319,WHITE);

    // Title with black background
    LCD_Clear(0, startY, DISPLAY_WIDTH, BLOCK_HEIGHT, BLACK);
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Taking %s's vitals...", patient->name);
    LCD_DrawStringColor(25, startY + 20, buffer, WHITE, BLACK);
    startY += BLOCK_HEIGHT;

    LCD_DrawString(40, startY+110, (uint8_t*) "Saving temperature...");

    // Put code to display readings here
}

void DisplayDispensingMedication(PatientDetails *patient) {
	uint16_t startY = 0; // Starting Y position
    LCD_Clear(0,0,239,319,WHITE);

    // Title with black background
    LCD_Clear(0, startY, DISPLAY_WIDTH, BLOCK_HEIGHT, BLACK);
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Dispensing %s's pills...", patient->name);
    LCD_DrawStringColor(10, startY + 20, buffer, WHITE, BLACK);
    startY += BLOCK_HEIGHT;

    char buffer2[100];
    snprintf(buffer2, sizeof(buffer2), "White x %d", patient->medicine1Pills);
    LCD_DrawStringColor(80, startY+100, buffer2, MAGENTA, WHITE);
    snprintf(buffer2, sizeof(buffer2), "Black x %d", patient->medicine2Pills);
    LCD_DrawStringColor(80, startY+120, buffer2, MAGENTA, WHITE);
}

void blinkRed() {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // Red on
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   // Green off
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   // Blue off
    DelayMicroseconds(100000);                             // Wait for 100 ms for visibility
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // Red off
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   // Green off
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   // Blue off
    DelayMicroseconds(100000);                             // Wait for 100 ms for visibility
}
