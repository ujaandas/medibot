/*
 * FIFOController.h
 *
 *  Created on: Nov 20, 2024
 *      Author: ooj
 */

#ifndef __FIFO_CONTROLLER_H
#define __FIFO_CONTROLLER_H

#include "stm32f1xx_hal.h"
#include "GPIOPin.h"

class FIFOController {
public:
    FIFOController(GPIOPin cs, GPIOPin wrst, GPIOPin rrst, GPIOPin rclk, GPIOPin we);

    // FIFO Control Methods
    void setCs(bool high) const;
    void setWrst(bool high) const;
    void setRrst(bool high) const;
    void setRclk(bool high) const;
    void setWe(bool high) const;

    // FIFO Operations
    uint16_t readPixel();
    void prepareFIFO();

private:
    GPIOPin cs;
    GPIOPin wrst;
    GPIOPin rrst;
    GPIOPin rclk;
    GPIOPin we;
};

#endif /* __FIFO_CONTROLLER_H */
