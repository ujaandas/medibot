/*
 * Camera.cpp
 *
 *  Created on: 19 Nov 2024
 *      Author: ooj
 */

#include "Camera.h"

struct RegConfig {
        uint8_t address;
        uint8_t value;
    };

// Define the configuration array
static constexpr RegConfig CONFIG[] =
{
	{CLKRC,     0x00}, /*clock config*/
	{COM7,      0x46}, /*QVGA RGB565 */
	{HSTART,    0x3f},
	{HSIZE,     0x50},
	{VSTRT,     0x03},
	{VSIZE,     0x78},
	{HREF,      0x00},
	{HOutSize,  0x50},
	{VOutSize,  0x78},
	{EXHCH,     0x00},

	/*DSP control*/
	{TGT_B,     0x7f},
	{FixGain,   0x09},
	{AWB_Ctrl0, 0xe0},
	{DSP_Ctrl1, 0xff},
	{DSP_Ctrl2, 0x20},
	{DSP_Ctrl3,	0x00},
	{DSP_Ctrl4, 0x00},

	/*AGC AEC AWB*/
	{COM8,		  0xf0},
	{COM4,		  0x81}, /*Pll AEC CONFIG*/
	{COM6,		  0xc5},
	{COM9,		  0x21},
	{BDBase,	  0xFF},
	{BDMStep,	  0x01},
	{AEW,		    0x34},
	{AEB,		    0x3c},
	{VPT,		    0xa1},
	{EXHCL,		  0x00},
	{AWBCtrl3,  0xaa},
	{COM8,		  0xff},
	{AWBCtrl1,  0x5d},

	{EDGE1,		  0x0a},
	{DNSOff,	  0x01},
	{EDGE2,		  0x01},
	{EDGE3,		  0x01},

	{MTX1,		  0x5f},
	{MTX2,		  0x53},
	{MTX3,		  0x11},
	{MTX4,		  0x1a},
	{MTX5,		  0x3d},
	{MTX6,		  0x5a},
	{MTX_Ctrl,  0x1e},

	// Image
	{BRIGHT,	  0x00},
	{CNST,		  0x10},
	{USAT,		  0x85},
	{VSAT,		  0x85},
	{UVADJ0,	  0x81},
	{SDE,		  0x02},

    /*GAMMA config*/
	{GAM1,		  0x0c},
	{GAM2,		  0x16},
	{GAM3,		  0x2a},
	{GAM4,		  0x4e},
	{GAM5,		  0x61},
	{GAM6,		  0x6f},
	{GAM7,		  0x7b},
	{GAM8,		  0x86},
	{GAM9,		  0x8e},
	{GAM10,		  0x97},
	{GAM11,		  0xa4},
	{GAM12,		  0xaf},
	{GAM13,		  0xc5},
	{GAM14,		  0xd7},
	{GAM15,		  0xe8},
	{SLOP,		  0x20},

	{HUECOS,	  0x80},
	{HUESIN,	  0x80},
	{DSPAuto,	  0xFF},
	{DM_LNL,	  0x00},
	{BDBase,	  0x99},
	{BDMStep,	  0x03},
	{LC_RADI,	  0x00},
	{LC_COEF,	  0x13},
	{LC_XC,		  0x08},
	{LC_COEFB,  0x14},
	{LC_COEFR,  0x17},
	{LC_CTR,	  0x05},

	{COM3,		  0xd0},/*Horizontal mirror image*/

	/*night mode auto frame rate control*/
//	{COM5,		0xf5},	 /*auto reduce rate*/
	{COM5,		0x31},	/*no auto*/
};

static constexpr size_t CONFIG_SIZE = sizeof(CONFIG) / sizeof(CONFIG[0]);
static constexpr uint8_t SENSOR_ID = 0x21;

Camera::Camera(const SCCBController& sccbController, const FIFOController& fifoController)
	: initialized(false), sccbController(sccbController), fifoController(fifoController) {
	this->init();
}

void Camera::init() {
	LCD_DrawString(50, 150, (uint8_t*) "Initializing camera...");
    // Reset sensor
    if (!writeSensorReg(COM7, GAM3)) {
    	LCD_DrawString(50, 200, (uint8_t*) "Reset sensor failed!");
    	return;
    }
    HAL_Delay(10); // Wait for reset to complete

    // Verify sensor ID
    if (!verifySensorID()) {
    	LCD_DrawString(50, 200, (uint8_t*) "Verify sensor failed!");
    	return;
    }

    // Configure sensor registers
    for (size_t i = 0; i < CONFIG_SIZE; i++) {
		if (!writeSensorReg(CONFIG[i].address, CONFIG[i].value)) {
			LCD_DrawString(50, 200, (uint8_t*) "Configure sensor failed!");
			return;
		}
	}

    initialized = true;
    LCD_DrawString(50, 170, (uint8_t*) "Camera init success!");
}

bool Camera::isInitialized() {
	return initialized;
}

void Camera::displayImage() {
    if (!initialized) return;

	uint16_t i, j;
	uint16_t Camera_Data;

	fifoController.prepareFIFO();
	LCD_Cam_Gram();

	for(i = 0; i < 240; i++)
	{
		for(j = 0; j < 320; j++)
		{
			Camera_Data = fifoController.readPixel();
			LCD_Write_Data(Camera_Data);
		}
	}
	HAL_Delay(1000);
}


bool Camera::writeSensorReg(uint8_t addr, uint8_t data) {
    return sccbController.writeByte(addr, data);
}

bool Camera::readSensorReg(uint8_t addr, uint8_t& data) {
    return sccbController.readBytes(&data, 1, addr);
}

bool Camera::verifySensorID() {
    uint8_t sensorId;
    if (!readSensorReg(VER, sensorId)) { // change back to 0x0b if err
        return false;
    }
    return sensorId == SENSOR_ID;
}

void Camera::vsyncHandler() {
	if( vsync == 0 )
	{
		fifoController.setWrst(false);
		fifoController.setWe(false);

		vsync = 1;
		fifoController.setWe(true);
		fifoController.setWrst(true);
	}
	else if( vsync == 1 )
	{
		fifoController.setWe(false);
		vsync = 2;
	}
}

bool Camera::isColorAtPoint(uint16_t x, uint16_t y, uint16_t targetColor) {
    if (!initialized) return false;

    // Validate coordinates
    if (x >= 320 || y >= 240) return false;

    // Read pixel data at (x, y)
    uint16_t Camera_Data;
    fifoController.prepareFIFO();
    for (uint16_t i = 0; i <= y; i++) {
        for (uint16_t j = 0; j < 320; j++) {
            Camera_Data = fifoController.readPixel();
            if (i == y && j == x) {
                // Check if the pixel matches the target color
                return Camera_Data == targetColor;
            }
        }
    }

    return false;
}

