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

extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim3;
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
SCCBController sccb(GPIOC, CAM_SCL_Pin, CAM_SDA_Pin);
FIFOController fifo(
		  {GPIOA, CAM_CS_Pin}, // cs
		  {GPIOC, CAM_WRST_Pin}, // wrst
		  {GPIOA, CAM_RRST_Pin}, // rrst
		  {GPIOC, CAM_RCLK_Pin}, // rclk
		  {GPIOD, CAM_WE_Pin});
Camera camera(sccb, fifo);

int mymain(void)
{
  TOUCHSCREEN_CS_DISABLE();
  LCD_INIT();

  StepperMotor stepper(GPIOA, STP_1_Pin, STP_2_Pin, STP_3_Pin, STP_4_Pin, &htim1);
//  ServoMotor armServo(&htim3, TIM_CHANNEL_3);

  camera.init();

  camera.vsync = 0;
  while (1)
  {
	  if (camera.vsync == 2)
	  {
		  camera.displayImage();
		  camera.vsync = 0;
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

