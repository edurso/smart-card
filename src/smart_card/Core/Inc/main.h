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
#include "stm32l4xx_hal.h"

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
#define RFID_SS_Pin GPIO_PIN_0
#define RFID_SS_GPIO_Port GPIOA
#define SPI_SCK_Pin GPIO_PIN_1
#define SPI_SCK_GPIO_Port GPIOA
#define DBG_TX_Pin GPIO_PIN_2
#define DBG_TX_GPIO_Port GPIOA
#define DBG_RX_Pin GPIO_PIN_3
#define DBG_RX_GPIO_Port GPIOA
#define LCD_BL_Pin GPIO_PIN_4
#define LCD_BL_GPIO_Port GPIOA
#define RFID_RST_Pin GPIO_PIN_5
#define RFID_RST_GPIO_Port GPIOA
#define SPI_MISO_Pin GPIO_PIN_6
#define SPI_MISO_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_7
#define SPI_MOSI_GPIO_Port GPIOA
#define LCD_RST_Pin GPIO_PIN_0
#define LCD_RST_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_1
#define IMU_INT_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI1_IRQn
#define PWM_BUZ_Pin GPIO_PIN_8
#define PWM_BUZ_GPIO_Port GPIOA
#define I2C_SCL_Pin GPIO_PIN_9
#define I2C_SCL_GPIO_Port GPIOA
#define I2C_SDA_Pin GPIO_PIN_10
#define I2C_SDA_GPIO_Port GPIOA
#define LED_ERR_Pin GPIO_PIN_11
#define LED_ERR_GPIO_Port GPIOA
#define LED_SUC_Pin GPIO_PIN_12
#define LED_SUC_GPIO_Port GPIOA
#define STM_LED_Pin GPIO_PIN_3
#define STM_LED_GPIO_Port GPIOB
#define TS_CS_Pin GPIO_PIN_4
#define TS_CS_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_5
#define LCD_RS_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_7
#define LCD_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
