/*
 * FIFOController.h
 *
 *  Created on: Nov 20, 2024
 *      Author: ooj
 */

#ifndef __FIFO_CONTROLLER_H
#define __FIFO_CONTROLLER_H

#include "stm32f1xx_hal.h"

struct GPIOPin {
    GPIO_TypeDef* port;
    uint16_t pin;

    void setHigh() const;
    void setLow() const;
};

class FIFOController {
public:
    FIFOController(GPIOPin cs, GPIOPin wrst, GPIOPin rrst, GPIOPin rclk, GPIOPin we);

    // FIFO Control Methods
    void csHigh();
    void csLow();

    void wrstHigh();
    void wrstLow();

    void rrstHigh();
    void rrstLow();

    void rclkHigh();
    void rclkLow();

    void weHigh();
    void weLow();

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
