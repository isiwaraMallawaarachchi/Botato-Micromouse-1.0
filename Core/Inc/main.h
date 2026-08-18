/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TOF_LEFT_INT_Pin GPIO_PIN_2
#define TOF_LEFT_INT_GPIO_Port GPIOA
#define TOF_LEFT_INT_EXTI_IRQn EXTI2_IRQn
#define TOF_LF_INT_Pin GPIO_PIN_3
#define TOF_LF_INT_GPIO_Port GPIOA
#define TOF_LF_INT_EXTI_IRQn EXTI3_IRQn
#define TOF_FRONT_INT_Pin GPIO_PIN_4
#define TOF_FRONT_INT_GPIO_Port GPIOA
#define TOF_FRONT_INT_EXTI_IRQn EXTI4_IRQn
#define BTN1_Pin GPIO_PIN_5
#define BTN1_GPIO_Port GPIOA
#define BTN2_Pin GPIO_PIN_6
#define BTN2_GPIO_Port GPIOA
#define XSHUT_RF_Pin GPIO_PIN_7
#define XSHUT_RF_GPIO_Port GPIOA
#define TOF_RF_INT_Pin GPIO_PIN_0
#define TOF_RF_INT_GPIO_Port GPIOB
#define TOF_RF_INT_EXTI_IRQn EXTI0_IRQn
#define TOF_RIGHT_INT_Pin GPIO_PIN_1
#define TOF_RIGHT_INT_GPIO_Port GPIOB
#define TOF_RIGHT_INT_EXTI_IRQn EXTI1_IRQn
#define XSHUT_RIGHT_Pin GPIO_PIN_10
#define XSHUT_RIGHT_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_12
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_13
#define AIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_14
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_15
#define BIN2_GPIO_Port GPIOB
#define XSHUT_LEFT_Pin GPIO_PIN_10
#define XSHUT_LEFT_GPIO_Port GPIOA
#define XSHUT_LF_Pin GPIO_PIN_4
#define XSHUT_LF_GPIO_Port GPIOB
#define XSHUT_FRONT_Pin GPIO_PIN_5
#define XSHUT_FRONT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
