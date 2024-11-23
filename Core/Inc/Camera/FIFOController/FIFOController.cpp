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
void FIFOController::setCs(bool high) const {
	if (high) {
		cs.setHigh();
	} else {
		cs.setLow();
	}
}

void FIFOController::setWrst(bool high) const {
	if (high) {
		wrst.setHigh();
	} else {
		wrst.setLow();
	}
}

void FIFOController::setRrst(bool high) const {
	if (high) {
		rrst.setHigh();
	} else {
		rrst.setLow();
	}
}

void FIFOController::setRclk(bool high) const {
	if (high) {
		rclk.setHigh();
	} else {
		rclk.setLow();
	}
}

void FIFOController::setWe(bool high) const {
	if (high) {
		we.setHigh();
	} else {
		we.setLow();
	}
}

// FIFO Operations
uint16_t FIFOController::readPixel() {
    uint16_t RGB565 = 0;
    setRclk(false);
    RGB565 = (GPIOB->IDR) & 0xff00;
    setRclk(true);
    setRclk(false);
    RGB565 |= (GPIOB->IDR >> 8) & 0x00ff;
    setRclk(true);
    return RGB565;
}

void FIFOController::prepareFIFO() {
    setRrst(false);
    setRclk(false);
    setRclk(true);
    setRrst(true);
    setRclk(false);
    setRclk(true);
}
