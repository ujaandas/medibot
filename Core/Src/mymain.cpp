#include "main.h"

extern "C" int mymain(void);

#include <stdio.h>
#include "Screen/touchscreenDriver.h"
#include "Screen/lcd.h"
#include "StepperMotor/StepperMotor.h"
#include "CupServo/CupServo.h"
#include "ServoMotor/ServoMotor.h"
#include "Camera/Camera.h"
#include "Camera/SCCBController/SCCBController.h"
#include "Camera/FIFOController/FIFOController.h"
#include "Detector/Detector.h"
#include "LDR/LDR.h"
#include "Temperature/Temperature.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern uint8_t Ov7725_vsync;
extern UART_HandleTypeDef huart1;
extern void handleK2BtnPress(int cup, TIM_HandleTypeDef *timer, uint16_t timerChannel);
extern void moveBasePlate(TIM_HandleTypeDef *timer, int steps, uint16_t delay, bool clockwise);

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#define NUM_PATIENTS 4

PatientDetails patients[NUM_PATIENTS] = {
    {"Armaan", 72, 98, 36.5, 2, 0},
    {"Ujaan", 75, 95, 37.0, 1, 2},
    {"Tim", 70, 99, 37.1, 0, 2},
    {"Fox", 68, 99, 36.8, 0, 1},
};

SCCBController sccb(GPIOC, CAM_SCL_Pin, CAM_SDA_Pin);
FIFOController fifo(
    {GPIOA, CAM_CS_Pin},
    {GPIOC, CAM_WRST_Pin},
    {GPIOA, CAM_RRST_Pin},
    {GPIOC, CAM_RCLK_Pin},
    {GPIOD, CAM_WE_Pin});
Camera camera(sccb, fifo);
uint16_t targetColours[] = {0x8537, 0x3207};
void colourDetectedHandler(int32_t detectedColour)
{
    if (detectedColour == targetColours[0])
    {
        LCD_DrawStringColor(100, 200, "White!", RED, WHITE);
        handleK2BtnPress(0, &htim3, TIM_CHANNEL_4);
    }
    else if (detectedColour == targetColours[1])
    {
        LCD_DrawStringColor(100, 200, "Black!", RED, WHITE);
        handleK2BtnPress(1, &htim3, TIM_CHANNEL_4);
    }
    else if (detectedColour == targetColours[0] * -2)
    {
        LCD_DrawStringColor(100, 200, "Double white!", RED, WHITE);
        handleK2BtnPress(2, &htim3, TIM_CHANNEL_4);
    }
    else if (detectedColour == targetColours[1] * -2)
    {
        LCD_DrawStringColor(100, 200, "Double black!", RED, WHITE);
        handleK2BtnPress(2, &htim3, TIM_CHANNEL_4);
    }
    else if (detectedColour == targetColours[0] * -3)
    {
        LCD_DrawStringColor(100, 200, "White done!", RED, WHITE);
        handleK2BtnPress(2, &htim3, TIM_CHANNEL_4);
    }
    else if (detectedColour == targetColours[1] * -3)
    {
        LCD_DrawStringColor(100, 200, "Black done!", RED, WHITE);
        handleK2BtnPress(2, &htim3, TIM_CHANNEL_4);
    }
    else if (detectedColour == 420)
    {
        LCD_Clear(0, 0, 319, 239, WHITE);
        LCD_DrawString(50, 240, (uint8_t *)"All pills dispensed");
        moveBasePlate(&htim1, 0, 0, false);
    }
    else
    {
        char buf[12];
        sprintf(buf, "0x%04X", detectedColour);
        LCD_DrawStringColor(100, 200, buf, RED, WHITE);
        handleK2BtnPress(2, &htim3, TIM_CHANNEL_4);
    }
}

int mymain(void)
{
    TOUCHSCREEN_CS_DISABLE();
    LCD_INIT();

    uint8_t selectedPatientIndex = 0;
    uint8_t selectedOptionIndex = 0; // 0 for vitals, 1 for medication
    char buf[12];

    LDR ldr(&hadc1);
    Temperature tempSensor(&hadc2);

    camera.init();
    ServoMotor armServo(&htim3, TIM_CHANNEL_3);
    camera.vsync = 0;

    LCD_Clear(0, 0, 239, 319, WHITE);
    LCD_DrawStringColor(40, 140, "Welcome to MediMate!", RED, WHITE);
    CycleLedGradient(500);

    while (1)
    {
        // Main loop to select patients and options
        while (1)
        {
            // 1A: DISPLAY patients
            DisplayPatientSelection(patients, NUM_PATIENTS);
            while (isScreenTouched == 0)
            {
                // Wait for touch
            }

            // 1B: SELECT patient
            int isPatientSelected = 0;
            Coordinate patientCoords;
            if (ReadSmoothedCoordinates(&patientCoords))
            {
                uint16_t startY = BLOCK_HEIGHT + GAP; // Start after the title block

                for (uint16_t i = 0; i < NUM_PATIENTS; i++)
                {
                    if (patientCoords.y >= startY && patientCoords.y < (startY + BLOCK_HEIGHT))
                    {
                        selectedPatientIndex = i; // Set the selected patient index
                        isPatientSelected = 1;
                        break;
                    }
                    startY += BLOCK_HEIGHT + GAP; // Move to the next block
                }

                if (isPatientSelected)
                {
                    break; // Exit the loop after selection attempt
                }
            }
        }

        while (1)
        {
            // 2A: DISPLAY options
            DisplayPatientOptions(&patients[selectedPatientIndex]);
            while (isScreenTouched == 0)
            {
                // Wait for touch
            }

            // 2B: SELECT option
            int isOptionSelected = 0;
            Coordinate optionCoords;
            if (ReadSmoothedCoordinates(&optionCoords))
            {
                uint16_t startY = BLOCK_HEIGHT + GAP; // Start after the title block

                for (uint16_t i = 0; i < 2; i++)
                {
                    if (optionCoords.y >= startY && optionCoords.y < (startY + BLOCK_HEIGHT))
                    {
                        selectedOptionIndex = i; // Set the selected option index
                        isOptionSelected = 1;
                        break;
                    }
                    startY += BLOCK_HEIGHT + GAP; // Move to the next block
                }

                if (isOptionSelected)
                {
                    break; // Exit the loop after selection attempt
                }
            }
        }

        if (selectedOptionIndex == 0)
        { // Take vitals
            DisplayTakingVitals(&patients[selectedPatientIndex]);
            // Put code to take measurements here
            uint8_t time = 244;
            while (time-- > 0)
            {
                tempSensor.read();
                sprintf(buf, "%4.1f", tempSensor.getTemperatureCelsius());
                LCD_DrawStringColor(10, 10, buf, BLACK, WHITE);
                HAL_Delay(100);
            }
            // Instead of break, continue to main loop
        }
        else if (selectedOptionIndex == 1)
        { // Dispense medication
            DisplayDispensingMedication(&patients[selectedPatientIndex]);
            HAL_Delay(3000);
            // Put code to dispense medication here
            uint8_t desiredCount[] = {patients[selectedPatientIndex].medicine1Pills, patients[selectedPatientIndex].medicine2Pills};
            Detector detector(camera, armServo, targetColours, desiredCount, 2, colourDetectedHandler);

            while (1)
            {
                // Calibrate armServo first using "armServo.spinTo(90);"
                // Once calibrated, start dispensing medications (pill 1 = BLACK, pill 2 = WHITE) + checking LDR
                if (camera.vsync == 2 && !detector.done)
                {
                    moveBasePlate(&htim1, 16, 3000, false);
                    detector.calibrate(220, 110, 100);
                    if (detector.isCalibrated())
                    {
                        detector.displayImage(220, 110, 100);
                    }
                    camera.vsync = 0;

                    // ldr stuff
                    ldr.read();
                    sprintf(buf, "%4lu", ldr.getIntensity());
                    LCD_DrawStringColor(10, 270, "Checking for blockages...", BLACK, WHITE);
                    LCD_DrawStringColor(10, 290, "Current light intensity:", BLACK, WHITE);
                    LCD_DrawStringColor(200, 290, buf, BLACK, WHITE);
                    if (ldr.somethingBlocking(70, 2000))
                    {
                        LCD_Clear(0, 0, 239, 319, WHITE);
                        LCD_DrawStringColor(50, 150, "Blockage detected!", RED, WHITE);
                        LCD_DrawStringColor(70, 170, "Please reset", RED, WHITE);
                        break;
                    }
                }
                else
                {
                    moveBasePlate(&htim1, 0, 0, false);
                }
            }
            while (1)
            {
                blinkRed(); // Only blinks red once finished
            }
        }
        // Return to the main loop
    }
}

PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART1 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}
