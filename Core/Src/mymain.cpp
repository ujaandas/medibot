#include "main.h"

extern "C" int mymain(void);

#include <stdio.h>
#include "Screen/touchscreenDriver.h"
#include "Screen/lcd.h"
#include "StepperMotor/StepperMotor.h"
#include "CupServo/CupServo.h"
#include "Camera/Camera.h"
#include "Camera/Helper/CameraPins.h"

extern TIM_HandleTypeDef htim1;
extern uint8_t Ov7725_vsync;
extern UART_HandleTypeDef huart1;

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#define NUM_PATIENTS 4

PatientDetails patients[NUM_PATIENTS] = {
    {"Armaan", 72, 98, 36.5, 2, 0, 1},
    {"Ujaan", 75, 95, 37.0, 1, 2, 0},
    {"Tim", 70, 99, 37.1, 0, 2, 1},
	{"Fox", 68, 99, 36.8, 0, 1, 1},
};

int mymain(void)
{
  TOUCHSCREEN_CS_DISABLE();
  LCD_INIT();

  LCD_DrawStringColor(40, 140, "Welcome to MediMate!", RED, WHITE);
  CycleLedGradient(500);

  uint8_t selectedPatientIndex = 0;
  uint8_t selectedOptionIndex = 0; // 0 for vitals, 1 for medication

  while (1) {
       // 1A: DISPLAY patients
       DisplayPatientSelection(patients, NUM_PATIENTS);
       while (isScreenTouched == 0) {
           // Wait for touch
       }

       // 1B: SELECT patient
       int isPatientSelected = 0;
       Coordinate patientCoords;
       if (ReadSmoothedCoordinates(&patientCoords)) {
           uint16_t startY = BLOCK_HEIGHT + GAP; // Start after the title block

           for (uint16_t i = 0; i < NUM_PATIENTS; i++) {
               if (patientCoords.y >= startY && patientCoords.y < (startY + BLOCK_HEIGHT)) {
                   selectedPatientIndex = i; // Set the selected patient index
                   isPatientSelected = 1;
                   break;
               }
               startY += BLOCK_HEIGHT + GAP; // Move to the next block
           }


           if (isPatientSelected){
         	  break; // Exit the loop after selection attempt
           }
       }
   }

  while (1){
      // 2A: DISPLAY options
  	  DisplayPatientOptions(&patients[selectedPatientIndex]);
      while (isScreenTouched == 0) {
          // Wait for touch
      }

      // 2B: SELECT option
      int isOptionSelected = 0;
      Coordinate optionCoords;
      if (ReadSmoothedCoordinates(&optionCoords)) {
          uint16_t startY = BLOCK_HEIGHT + GAP; // Start after the title block

          for (uint16_t i = 0; i < 2; i++) {
              if (optionCoords.y >= startY && optionCoords.y < (startY + BLOCK_HEIGHT)) {
            	  selectedOptionIndex = i; // Set the selected patient index
                  isOptionSelected = 1;
                  break;
              }
              startY += BLOCK_HEIGHT + GAP; // Move to the next block
          }


          if (isOptionSelected){
        	  break; // Exit the loop after selection attempt
          }
      }
  }

      if (selectedOptionIndex == 0){ // Take vitals
    	  DisplayTakingVitals(&patients[selectedPatientIndex]);
    	  // Put code to take measurements here
      } else if (selectedOptionIndex == 1){ // Dispense medication
    	  DisplayDispensingMedication(&patients[selectedPatientIndex]);
    	  // Put code to dispense medication here
      }

//  StepperMotor stepper(GPIOA, STP_1_Pin, STP_2_Pin, STP_3_Pin, STP_4_Pin, &htim1);

//  LCD_Clear (50, 80, 140, 70, RED);
//  LCD_DrawString(75, 100, (uint8_t*)"CAMERA DEMO");
//  	HAL_Delay(2000);
//
//  Camera camera(GPIOC, GPIO_PIN_6, GPIO_PIN_7);
//
//  LCD_DrawString(50, 150, (uint8_t*) "Initializing camera...");
//
//  if (!camera.init()) {
//	  LCD_DrawString(50, 170, (uint8_t*) "Camera init failed!");
//  } else {
//	  LCD_DrawString(50, 170, (uint8_t*) "Camera init success!");
//  }
//
//
//  Ov7725_vsync = 0;
//	while (1)
//	{
//		if (Ov7725_vsync == 2)
//		{
//			FIFO_PREPARE;
//			camera.displayImage();
//			Ov7725_vsync = 0;
//		}
//	}

  return 0;
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

