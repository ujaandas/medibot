/*
 * GPIOPin.h
 *
 *  Created on: 24 Nov 2024
 *      Author: ooj
 */

#ifndef __GPIO_PIN_H
#define __GPIO_PIN_H

struct GPIOPin {
    GPIO_TypeDef* port;
    uint16_t pin;

    void setHigh() const;
    void setLow() const;
};


#endif /* __GPIO_PIN_H */
