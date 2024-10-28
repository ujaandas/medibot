
#include "main.h"

extern "C" int mymain(void);

#include <Screen/lcd.h>
#include <stdio.h>
#include <StepperMotor/StepperMotor.h>
#include <ServoMotor/ServoMotor.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

int mymain(void)
{
  LCD_INIT();

  char name[] = "DAS, Ujaan";
  LCD_DrawString(0, 0, name);

  StepperMotor stepper(GPIOA, STP_1_Pin, STP_2_Pin, STP_3_Pin, STP_4_Pin, &htim1);
  ServoMotor servo(&htim3, TIM_CHANNEL_1);

  while (1)
  {
	  stepper.makeSteps(256, 300, true);
	  servo.spinTo(90);
	  HAL_Delay(100);
//	  servo.spinTo(0);
	  stepper.makeSteps(128, 300, false);
//	  servo.spinTo(180);


  }

  return 0;
}
