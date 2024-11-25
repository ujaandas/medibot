/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define K2_BTN_Pin GPIO_PIN_13
#define K2_BTN_GPIO_Port GPIOC
#define K2_BTN_EXTI_IRQn EXTI15_10_IRQn
#define CAM_RRST_Pin GPIO_PIN_2
#define CAM_RRST_GPIO_Port GPIOA
#define CAM_CS_Pin GPIO_PIN_3
#define CAM_CS_GPIO_Port GPIOA
#define STP_2_Pin GPIO_PIN_5
#define STP_2_GPIO_Port GPIOA
#define STP_3_Pin GPIO_PIN_6
#define STP_3_GPIO_Port GPIOA
#define STP_4_Pin GPIO_PIN_7
#define STP_4_GPIO_Port GPIOA
#define CAM_WRST_Pin GPIO_PIN_4
#define CAM_WRST_GPIO_Port GPIOC
#define CAM_RCLK_Pin GPIO_PIN_5
#define CAM_RCLK_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_0
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_1
#define LED_B_GPIO_Port GPIOB
#define LCD_BL_Pin GPIO_PIN_12
#define LCD_BL_GPIO_Port GPIOD
#define LCD_DB_Pin GPIO_PIN_13
#define LCD_DB_GPIO_Port GPIOD
#define CAM_SCL_Pin GPIO_PIN_6
#define CAM_SCL_GPIO_Port GPIOC
#define CAM_SDA_Pin GPIO_PIN_7
#define CAM_SDA_GPIO_Port GPIOC
#define STP_1_Pin GPIO_PIN_12
#define STP_1_GPIO_Port GPIOC
#define CAM_WE_Pin GPIO_PIN_3
#define CAM_WE_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_5
#define LED_R_GPIO_Port GPIOB
#define LCD_TP_Pin GPIO_PIN_0
#define LCD_TP_GPIO_Port GPIOE
#define LCD_RST_Pin GPIO_PIN_1
#define LCD_RST_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
