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
#define ACCEL_POWER_Pin GPIO_PIN_0
#define ACCEL_POWER_GPIO_Port GPIOC
#define ACCEL_X_Pin GPIO_PIN_1
#define ACCEL_X_GPIO_Port GPIOC
#define ACCEL_Y_Pin GPIO_PIN_2
#define ACCEL_Y_GPIO_Port GPIOC
#define ACCEL_Z_Pin GPIO_PIN_3
#define ACCEL_Z_GPIO_Port GPIOC
#define COOLER_SHORT_Pin GPIO_PIN_1
#define COOLER_SHORT_GPIO_Port GPIOA
#define COOLER_CURRENT_Pin GPIO_PIN_2
#define COOLER_CURRENT_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOA
#define SD_SCK_Pin GPIO_PIN_5
#define SD_SCK_GPIO_Port GPIOA
#define SD_MISO_Pin GPIO_PIN_6
#define SD_MISO_GPIO_Port GPIOA
#define SD_MOSI_Pin GPIO_PIN_7
#define SD_MOSI_GPIO_Port GPIOA
#define SD_DETECT_Pin GPIO_PIN_4
#define SD_DETECT_GPIO_Port GPIOC
#define BUTTON_Pin GPIO_PIN_5
#define BUTTON_GPIO_Port GPIOC
#define V_BAT_Pin GPIO_PIN_1
#define V_BAT_GPIO_Port GPIOB
#define BUCK_Pin GPIO_PIN_12
#define BUCK_GPIO_Port GPIOB
#define LD1_Pin GPIO_PIN_7
#define LD1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOC
#define TEMP_SDI_Pin GPIO_PIN_9
#define TEMP_SDI_GPIO_Port GPIOC
#define TEMP_SCK_Pin GPIO_PIN_8
#define TEMP_SCK_GPIO_Port GPIOA
#define SWD_IO_Pin GPIO_PIN_13
#define SWD_IO_GPIO_Port GPIOA
#define SWD_CLK_Pin GPIO_PIN_14
#define SWD_CLK_GPIO_Port GPIOA
#define SWD_O_Pin GPIO_PIN_3
#define SWD_O_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
