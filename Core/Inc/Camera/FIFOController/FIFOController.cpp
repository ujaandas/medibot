#include "FIFOController.h"

void GPIOPin::setHigh() const {
    port->BSRR = pin;
}

void GPIOPin::setLow() const {
    port->BRR = pin;
}

FIFOController::FIFOController(GPIOPin cs, GPIOPin wrst, GPIOPin rrst, GPIOPin rclk, GPIOPin we)
    : cs(cs), wrst(wrst), rrst(rrst), rclk(rclk), we(we) {}

// FIFO Control Methods
void FIFOController::csHigh() { cs.setHigh(); }
void FIFOController::csLow() { cs.setLow(); }

void FIFOController::wrstHigh() { wrst.setHigh(); }
void FIFOController::wrstLow() { wrst.setLow(); }

void FIFOController::rrstHigh() { rrst.setHigh(); }
void FIFOController::rrstLow() { rrst.setLow(); }

void FIFOController::rclkHigh() { rclk.setHigh(); }
void FIFOController::rclkLow() { rclk.setLow(); }

void FIFOController::weHigh() { we.setHigh(); }
void FIFOController::weLow() { we.setLow(); }

// FIFO Operations
uint16_t FIFOController::readPixel() {
    uint16_t RGB565 = 0;
    rclkLow();
    RGB565 = (GPIOB->IDR) & 0xff00;
    rclkHigh();
    rclkLow();
    RGB565 |= (GPIOB->IDR >> 8) & 0x00ff;
    rclkHigh();
    return RGB565;
}

void FIFOController::prepareFIFO() {
    rrstLow();
    rclkLow();
    rclkHigh();
    rrstHigh();
    rclkLow();
    rclkHigh();
}
