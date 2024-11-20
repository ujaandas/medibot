#ifndef __OV7725_PINS_H
#define __OV7725_PINS_H

#include "stm32f1xx_hal.h"

// FIFO Control Macros
#define FIFO_CS_H()     GPIOA->BSRR = GPIO_PIN_3
#define FIFO_CS_L()     GPIOA->BRR = GPIO_PIN_3

#define FIFO_WRST_H()   GPIOC->BSRR = GPIO_PIN_4
#define FIFO_WRST_L()   GPIOC->BRR = GPIO_PIN_4

#define FIFO_RRST_H()   GPIOA->BSRR = GPIO_PIN_2
#define FIFO_RRST_L()   GPIOA->BRR = GPIO_PIN_2

#define FIFO_RCLK_H()   GPIOC->BSRR = GPIO_PIN_5
#define FIFO_RCLK_L()   GPIOC->BRR = GPIO_PIN_5

#define FIFO_WE_H()     GPIOD->BSRR = GPIO_PIN_3
#define FIFO_WE_L()     GPIOD->BRR = GPIO_PIN_3

// FIFO Operations
#define READ_FIFO_PIXEL(RGB565) do { \
    RGB565 = 0; \
    FIFO_RCLK_L(); \
    RGB565 = (GPIOB->IDR) & 0xff00; \
    FIFO_RCLK_H(); \
    FIFO_RCLK_L(); \
    RGB565 |= (GPIOB->IDR >> 8) & 0x00ff; \
    FIFO_RCLK_H(); \
} while(0)

#define FIFO_PREPARE do { \
    FIFO_RRST_L(); \
    FIFO_RCLK_L(); \
    FIFO_RCLK_H(); \
    FIFO_RRST_H(); \
    FIFO_RCLK_L(); \
    FIFO_RCLK_H(); \
} while(0)

#endif
