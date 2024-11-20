/*
 * SCCB.cpp
 *
 *  Created on: 19 Nov 2024
 *      Author: ooj
 */

#include <Camera/SCCB/SCCB.h>
#include "Screen/lcd.h"
#include <stdio.h>

SCCB::SCCB(GPIO_TypeDef* port, uint16_t sclPin, uint16_t sdaPin)
    : port_(port), sclPin_(sclPin) , sdaPin_(sdaPin) {}

void SCCB::delay() const {
    volatile uint16_t i = DELAY_COUNT;
    while(i--);
}

void SCCB::setSCL(bool high) const {
    if (high) {
        port_->BSRR = sclPin_;
    } else {
        port_->BRR = sclPin_;
    }
}

void SCCB::setSDA(bool high) const {
    if (high) {
        port_->BSRR = sdaPin_;
    } else {
        port_->BRR = sdaPin_;
    }
}

bool SCCB::readSCL() const {
    return HAL_GPIO_ReadPin(port_, sclPin_) == GPIO_PIN_SET;
}

bool SCCB::readSDA() const {
	LCD_DrawString(30, 250, (uint8_t*) (HAL_GPIO_ReadPin(port_, sdaPin_) ? "true" : "false"));
    return HAL_GPIO_ReadPin(port_, sdaPin_) == GPIO_PIN_SET;
}

bool SCCB::start() {
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

void SCCB::stop() {
    setSCL(false);
    delay();
    setSDA(false);
    delay();
    setSCL(true);
    delay();
    setSDA(true);
    delay();
}

void SCCB::sendAck() {
    setSCL(false);
    delay();
    setSDA(false);
    delay();
    setSCL(true);
    delay();
    setSCL(false);
    delay();
}

void SCCB::sendNoAck() {
    setSCL(false);
    delay();
    setSDA(true);
    delay();
    setSCL(true);
    delay();
    setSCL(false);
    delay();
}

bool SCCB::waitAck() {
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

void SCCB::sendByte(uint8_t byte) {
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

uint8_t SCCB::receiveByte() {
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

bool SCCB::writeByte(uint16_t writeAddress, uint8_t data) {
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

bool SCCB::readBytes(uint8_t* pBuffer, uint16_t length, uint8_t readAddress) {
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
