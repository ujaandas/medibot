/*
 * SCCB.h
 *
 *  Created on: 19 Nov 2024
 *      Author: ooj
 */

#ifndef __SCCB_H
#define __SCCB_H

#include "stm32f1xx_hal.h"

class SCCBController {
	public:
		SCCBController(GPIO_TypeDef* port, uint16_t sclPin, uint16_t sdaPin);

		bool writeByte(uint16_t writeAddress, uint8_t sendByte);
		bool readBytes(uint8_t* pBuffer, uint16_t length, uint8_t readAddress);

	private:
		static constexpr uint8_t OV7725_ADDR = 0x42;
		static constexpr uint16_t DELAY_COUNT = 400;

		GPIO_TypeDef* const port;
		const uint16_t sclPin;
		const uint16_t sdaPin;

		void delay() const;
		bool start();
		void stop();
		void sendAck();
		void sendNoAck();
		bool waitAck();
		void sendByte(uint8_t byte);
		uint8_t receiveByte();

		// GPIO control methods
		void setSCL(bool high) const;
		void setSDA(bool high) const;
		bool readSCL() const;
		bool readSDA() const;
};


#endif /* __SCCB_H */
