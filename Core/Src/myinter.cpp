#include "main.h"
#include "stm32f1xx_it.h"

#include "Screen/lcd.h"
#include "CupServo/CupServo.h"
#include "Camera/Camera.h"
#include "StepperMotor/StepperMotor.h"

extern Camera camera;

//extern "C" void handleK2BtnInt(int cup, TIM_HandleTypeDef* timer, uint16_t timerChannel) {
////	char name[] = "CLICKED";
////	LCD_DrawString(0, 0, (uint8_t*)name);
//	CupServo servo(3, timer, timerChannel);
//	servo.selectCup(cup);
//}

void handleK2BtnPress(int cup, TIM_HandleTypeDef* timer, uint16_t timerChannel) {
//	char name[] = "CLICKED";
//	LCD_DrawString(0, 0, (uint8_t*)name);
	CupServo servo(3, timer, timerChannel);
	servo.selectCup(cup);
}

extern "C" void handleVsyncInterrupt() {
	camera.vsyncHandler();
}

void moveBasePlate(TIM_HandleTypeDef* timer, int steps, uint16_t delay, bool clockwise) {
	StepperMotor stepper({GPIOC, STP_1_Pin}, {GPIOC, STP_2_Pin}, {GPIOA, STP_3_Pin}, {GPIOA, STP_4_Pin}, timer);
	stepper.makeSteps(steps, delay, clockwise);
}
