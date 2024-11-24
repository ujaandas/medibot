#include "main.h"

extern "C" int mymain(void);

#include <stdio.h>
#include <StepperMotor/StepperMotor.h>
#include <CupServo/CupServo.h>
#include <LDR/LDR.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

void ConvertToDecimalString(uint32_t adcVal, char *buffer)
{
  sprintf(buffer, "%4lu", adcVal);
}

int mymain(void)
{
  TOUCHSCREEN_CS_DISABLE();
  LCD_INIT();

  LCD_DrawStringColor(40, 140, "Welcome to MediMate!", RED, WHITE);
  CycleLedGradient(500);

  uint8_t selectedPatientIndex = 0;
  uint8_t selectedOptionIndex = 0; // 0 for vitals, 1 for medication

  StepperMotor stepper(GPIOA, STP_1_Pin, STP_2_Pin, STP_3_Pin, STP_4_Pin, &htim1);
  CupServo servo(3, &htim3, TIM_CHANNEL_1);
  LDR ldr(&hadc1);
  char buf[12];

  while (1)
  {
    //	  stepper.makeSteps(256, 300, true);
    //	  HAL_Delay(100);
    stepper.makeSteps(256, 700, false);
    ldr.read();
    ConvertToDecimalString(ldr.getIntensity(), buf);
    LCD_DrawString(30, 30, buf);

    if (ldr.somethingPassed(10))
    {
      LCD_DrawString(50, 50, "Something detected");
      servo.selectCup(0);
    }
    else
    {
      LCD_DrawString(50, 50, "                    ");
      servo.selectCup(1);
    }
    HAL_Delay(1000);
  }

  return 0;
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
