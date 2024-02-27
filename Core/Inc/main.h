/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define Buildin_Led_Pin GPIO_PIN_13
#define Buildin_Led_GPIO_Port GPIOC
#define Throttle_Pin GPIO_PIN_0
#define Throttle_GPIO_Port GPIOA
#define Pressure_Pin GPIO_PIN_1
#define Pressure_GPIO_Port GPIOA
#define RPM_Pin GPIO_PIN_2
#define RPM_GPIO_Port GPIOA
#define CS_SD_Pin GPIO_PIN_4
#define CS_SD_GPIO_Port GPIOA
#define TEMP1_Pin GPIO_PIN_15
#define TEMP1_GPIO_Port GPIOA
#define TEMP2_Pin GPIO_PIN_3
#define TEMP2_GPIO_Port GPIOB
#define TEMP3_Pin GPIO_PIN_4
#define TEMP3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
