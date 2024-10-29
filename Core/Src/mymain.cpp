
#include "main.h"

extern "C" int mymain(void);

#include <Screen/lcd.h>
#include <stdio.h>
#include <StepperMotor/StepperMotor.h>
#include <CupServo/CupServo.h>

extern TIM_HandleTypeDef htim1;

int mymain(void)
{
  LCD_INIT();

  char name[] = "TEST";
  LCD_DrawString(0, 0, name);

  StepperMotor stepper(GPIOA, STP_1_Pin, STP_2_Pin, STP_3_Pin, STP_4_Pin, &htim1);

  while (1)
  {
//	  stepper.makeSteps(256, 300, true);
//	  HAL_Delay(100);
	  stepper.makeSteps(128, 300, false);
  }

  return 0;
}
