#ifndef __BSP_TOUCHSCREEN_LCD_H
#define __BSP_TOUCHSCREEN_LCD_H

#include "stm32f1xx_hal.h"

#define TOUCHSCREEN_EXTI_ACTIVE_LEVEL                     0
#define TOUCHSCREEN_EXTI_READ()                            HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4)

#define TOUCHSCREEN_CS_ENABLE()                            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET)
#define TOUCHSCREEN_CS_DISABLE()                           HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET)

#define TOUCHSCREEN_CLK_HIGH()                             HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET)
#define TOUCHSCREEN_CLK_LOW()                              HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET)

#define TOUCHSCREEN_MOSI_SET()                             HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET)
#define TOUCHSCREEN_MOSI_CLEAR()                           HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET)

#define TOUCHSCREEN_MISO_READ()                            HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)

#define TOUCHSCREEN_CHANNEL_X                               0x90
#define TOUCHSCREEN_CHANNEL_Y                               0xd0

#define BLOCK_HEIGHT 54 // Height of each block
#define GAP 10 // Gap between blocks
#define DISPLAY_WIDTH 239
#define DISPLAY_HEIGHT 319 // Assume a height that can fit all blocks

typedef struct
{
   uint16_t x;
   uint16_t y;
} Coordinate;

typedef struct
{
    char name[20];   // Patient's name
    uint16_t heartRate;           // Heart rate (in beats per minute)
    uint16_t bloodOxygen;         // Blood oxygen level (in percentage)
    float temperature;             // Temperature (in degrees Celsius)
    uint8_t medicine1Pills;       // Number of pills for medicine 1
    uint8_t medicine2Pills;       // Number of pills for medicine 2
    uint8_t medicine3Pills;       // Number of pills for medicine 3
} PatientDetails;

extern volatile uint8_t isScreenTouched;

// Function prototypes
void DelayMicroseconds(__IO uint32_t count);
void WriteCommand(uint8_t command);
uint16_t ReadCommand(void);
uint16_t ReadAdcValue(uint8_t channel);
void ReadAdcCoordinates(int16_t *xReading, int16_t *yReading);
Coordinate ConvertRawToCoordinates(int rawX, int rawY);
uint8_t ReadSmoothedCoordinates(Coordinate *screenCoordinate);
void ToggleLED(void);
void DisplayPatientSelection(PatientDetails *patients, uint8_t numPatients);
void DisplayPatientOptions(PatientDetails *patient);
void DisplayTakingVitals(PatientDetails *patient);
void DisplayDispensingMedication(PatientDetails *patient);
void CycleLedGradient(uint32_t durationMs);
void blinkRed();

#endif /* __BSP_TOUCHSCREEN_LCD_H */
