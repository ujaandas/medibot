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

					void setHigh() const {
						port->BSRR = pin;
					}

					void setLow() const {
						port->BRR = pin;
					}
		};

class FIFOController {
	public:
		FIFOController(GPIOPin cs, GPIOPin wrst, GPIOPin rrst, GPIOPin rclk, GPIOPin we)
			: cs(cs), wrst(wrst), rrst(rrst), rclk(rclk), we(we) {}

		// FIFO Control Methods
		void csHigh() { cs.setHigh(); }
		void csLow() { cs.setLow(); }

		void wrstHigh() { wrst.setHigh(); }
		void wrstLow() { wrst.setLow(); }

		void rrstHigh() { rrst.setHigh(); }
		void rrstLow() { rrst.setLow(); }

		void rclkHigh() { rclk.setHigh(); }
		void rclkLow() { rclk.setLow(); }

		void weHigh() { we.setHigh(); }
		void weLow() { we.setLow(); }

		// FIFO Operations
		uint16_t readPixel() {
			uint16_t RGB565 = 0;
			rclkLow();
			RGB565 = (GPIOB->IDR) & 0xff00;
			rclkHigh();
			rclkLow();
			RGB565 |= (GPIOB->IDR >> 8) & 0x00ff;
			rclkHigh();
			return RGB565;
		}

		void prepareFIFO() {
			rrstLow();
			rclkLow();
			rclkHigh();
			rrstHigh();
			rclkLow();
			rclkHigh();
		}

    private:
        GPIOPin cs;
        GPIOPin wrst;
        GPIOPin rrst;
        GPIOPin rclk;
        GPIOPin we;
}

#endif /* __FIFO_CONTROLLER_H */
