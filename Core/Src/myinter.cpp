#include "main.h"
#include "stm32f1xx_it.h"

#include "Screen/lcd.h"
#include "CupServo/CupServo.h"
#include "Camera/Camera.h"

extern Camera camera;

extern "C" void handleK2BtnPress(int cup, TIM_HandleTypeDef* timer, uint16_t timerChannel) {
//	char name[] = "CLICKED";
//	LCD_DrawString(0, 0, (uint8_t*)name);
//	CupServo servo(3, timer, timerChannel);
//	servo.selectCup(cup);
}

extern "C" void handleVsyncInterrupt() {
	camera.vsyncHandler();
}
