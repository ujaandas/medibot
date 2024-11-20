#include "main.h"

extern "C" int mymain(void);

#include <stdio.h>
#include "Screen/lcd.h"
#include "StepperMotor/StepperMotor.h"
#include "CupServo/CupServo.h"
#include "Camera/Camera.h"
#include "Camera/CameraPins.h"

extern TIM_HandleTypeDef htim1;
extern uint8_t Ov7725_vsync;
extern UART_HandleTypeDef huart1;

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

int mymain(void)
{
  setbuf(stdin,NULL);
  LCD_INIT();

//  StepperMotor stepper(GPIOA, STP_1_Pin, STP_2_Pin, STP_3_Pin, STP_4_Pin, &htim1);

  LCD_Clear (50, 80, 140, 70, RED);
  LCD_DrawString(75, 100, (uint8_t*)"CAMERA DEMO");
  	HAL_Delay(2000);

  Camera camera(GPIOC, GPIO_PIN_6, GPIO_PIN_7);

  LCD_DrawString(50, 150, (uint8_t*) "Initializing camera...");

  if (!camera.init()) {
	  LCD_DrawString(50, 170, (uint8_t*) "Camera init failed!");
  } else {
	  LCD_DrawString(50, 170, (uint8_t*) "Camera init success!");
  }


  Ov7725_vsync = 0;
	while (1)
	{
		if (Ov7725_vsync == 2)
		{
			FIFO_PREPARE;
			camera.displayImage();
			Ov7725_vsync = 0;
		}
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

