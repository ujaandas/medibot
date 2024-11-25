/*
 * SCCB.cpp
 *
 *  Created on: 19 Nov 2024
 *      Author: ooj
 */

#include <Camera/SCCBController/SCCBController.h>
#include "Screen/lcd.h"
#include <stdio.h>

SCCBController::SCCBController(GPIO_TypeDef* port, uint16_t sclPin, uint16_t sdaPin)
    : port(port), sclPin(sclPin) , sdaPin(sdaPin) {}

void SCCBController::delay() const {
    volatile uint16_t i = DELAY_COUNT;
    while(i--);
}

void SCCBController::setSCL(bool high) const {
    if (high) {
        port->BSRR = sclPin;
    } else {
        port->BRR = sclPin;
    }
}

void SCCBController::setSDA(bool high) const {
    if (high) {
        port->BSRR = sdaPin;
    } else {
        port->BRR = sdaPin;
    }
}

bool SCCBController::readSCL() const {
    return HAL_GPIO_ReadPin(port, sclPin) == GPIO_PIN_SET;
}

bool SCCBController::readSDA() const {
	LCD_DrawString(30, 250, (uint8_t*) (HAL_GPIO_ReadPin(port, sdaPin) ? "true" : "false"));
    return HAL_GPIO_ReadPin(port, sdaPin) == GPIO_PIN_SET;
}

bool SCCBController::start() {
    setSDA(true);
    setSCL(true);
    delay();

    if (!readSDA()) return false;

    setSDA(false);
    delay();

    if (readSDA()) return false;

    setSDA(false);
    delay();
    return true;
}

void SCCBController::stop() {
    setSCL(false);
    delay();
    setSDA(false);
    delay();
    setSCL(true);
    delay();
    setSDA(true);
    delay();
}

void SCCBController::sendAck() {
    setSCL(false);
    delay();
    setSDA(false);
    delay();
    setSCL(true);
    delay();
    setSCL(false);
    delay();
}

void SCCBController::sendNoAck() {
    setSCL(false);
    delay();
    setSDA(true);
    delay();
    setSCL(true);
    delay();
    setSCL(false);
    delay();
}

bool SCCBController::waitAck() {
    setSCL(false);
    delay();
    setSDA(true);
    delay();
    setSCL(true);
    delay();

    if (readSDA()) {
        setSCL(false);
        return false;
    }

    setSCL(false);
    return true;
}

void SCCBController::sendByte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
        setSCL(false);
        delay();
        setSDA((byte >> i) & 0x01);
        delay();
        setSCL(true);
        delay();
    }
    setSCL(false);
}

uint8_t SCCBController::receiveByte() {
    uint8_t byte = 0;
    setSDA(true);

    for (int i = 7; i >= 0; i--) {
        byte <<= 1;
        setSCL(false);
        delay();
        setSCL(true);
        delay();
        if (readSDA()) {
            byte |= 0x01;
        }
    }

    setSCL(false);
    return byte;
}

bool SCCBController::writeByte(uint16_t writeAddress, uint8_t data) {
    if (!start()) return false;

    sendByte(OV7725_ADDR);
    if (!waitAck()) {
        stop();
        LCD_DrawString(50, 230, (uint8_t*) "shit 1");
        return false;
    }

    sendByte(static_cast<uint8_t>(writeAddress & 0x00FF));
    if (!waitAck()) {
        stop();
        LCD_DrawString(50, 230, (uint8_t*) "shit 2");
        return false;
    }

    sendByte(data);
    if (!waitAck()) {
        stop();
        LCD_DrawString(50, 230, (uint8_t*) "shit 3");
        return false;
    }

    stop();
    return true;
}

bool SCCBController::readBytes(uint8_t* pBuffer, uint16_t length, uint8_t readAddress) {
    if (!start()) return false;

    sendByte(OV7725_ADDR);
    if (!waitAck()) {
        stop();
        return false;
    }

    sendByte(readAddress);
    if (!waitAck()) {
        stop();
        return false;
    }

    stop();

    if (!start()) return false;

    sendByte(OV7725_ADDR + 1);
    if (!waitAck()) {
        stop();
        return false;
    }

    while (length--) {
        *pBuffer = receiveByte();
        if (length == 0) {
            sendNoAck();
        } else {
            sendAck();
        }
        pBuffer++;
    }

    stop();
    return true;
}
