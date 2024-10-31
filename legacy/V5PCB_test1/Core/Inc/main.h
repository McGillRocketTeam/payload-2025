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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern volatile uint8_t flush_timer;
extern uint16_t line_written;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void myprintf(const char *fmt, ...);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define xsignal_Pin GPIO_PIN_1
#define xsignal_GPIO_Port GPIOC
#define ysignal_Pin GPIO_PIN_2
#define ysignal_GPIO_Port GPIOC
#define zsignal_Pin GPIO_PIN_3
#define zsignal_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOA
#define SD_SCK_Pin GPIO_PIN_5
#define SD_SCK_GPIO_Port GPIOA
#define SD_MISO_Pin GPIO_PIN_6
#define SD_MISO_GPIO_Port GPIOA
#define SD_MOSI_Pin GPIO_PIN_7
#define SD_MOSI_GPIO_Port GPIOA
#define LED_INIT_Pin GPIO_PIN_7
#define LED_INIT_GPIO_Port GPIOC
#define STLINK_UART_TX_Pin GPIO_PIN_10
#define STLINK_UART_TX_GPIO_Port GPIOC
#define STLINK_UART_RX_Pin GPIO_PIN_11
#define STLINK_UART_RX_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi1 // change to the spi connection you use

#define ADC_BUFFER_SIZE 4064

#define TERMINATION_TIME_MS 28800000 // 8 hours
//#define TERMINATION_TIME_MS 60000 // 1 minute to test termination
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
