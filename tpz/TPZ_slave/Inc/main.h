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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TAP10_DOWN_Pin GPIO_PIN_0
#define TAP10_DOWN_GPIO_Port GPIOC
#define TAP10_UP_Pin GPIO_PIN_1
#define TAP10_UP_GPIO_Port GPIOC
#define TAP11_DOWN_Pin GPIO_PIN_2
#define TAP11_DOWN_GPIO_Port GPIOC
#define TAP11_UP_Pin GPIO_PIN_3
#define TAP11_UP_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TAP0_DOWN_Pin GPIO_PIN_4
#define TAP0_DOWN_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TAP0_UP_Pin GPIO_PIN_6
#define TAP0_UP_GPIO_Port GPIOA
#define TAP1_DOWN_Pin GPIO_PIN_7
#define TAP1_DOWN_GPIO_Port GPIOA
#define TAP12_DOWN_Pin GPIO_PIN_4
#define TAP12_DOWN_GPIO_Port GPIOC
#define TAP12_UP_Pin GPIO_PIN_5
#define TAP12_UP_GPIO_Port GPIOC
#define TAP2_UP_Pin GPIO_PIN_0
#define TAP2_UP_GPIO_Port GPIOB
#define TAP3_DOWN_Pin GPIO_PIN_1
#define TAP3_DOWN_GPIO_Port GPIOB
#define TAP3_UP_Pin GPIO_PIN_2
#define TAP3_UP_GPIO_Port GPIOB
#define TAP7_UP_Pin GPIO_PIN_10
#define TAP7_UP_GPIO_Port GPIOB
#define TAP8_DOWN_Pin GPIO_PIN_12
#define TAP8_DOWN_GPIO_Port GPIOB
#define TAP8_UP_Pin GPIO_PIN_13
#define TAP8_UP_GPIO_Port GPIOB
#define TAP9_DOWN_Pin GPIO_PIN_14
#define TAP9_DOWN_GPIO_Port GPIOB
#define TAP9_UP_Pin GPIO_PIN_15
#define TAP9_UP_GPIO_Port GPIOB
#define TAP13_DOWN_Pin GPIO_PIN_9
#define TAP13_DOWN_GPIO_Port GPIOC
#define TAP1_UP_Pin GPIO_PIN_8
#define TAP1_UP_GPIO_Port GPIOA
#define TAP4_DOWN_Pin GPIO_PIN_9
#define TAP4_DOWN_GPIO_Port GPIOA
#define TAP15_DOWN_Pin GPIO_PIN_10
#define TAP15_DOWN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define TAP2_DOWN_Pin GPIO_PIN_15
#define TAP2_DOWN_GPIO_Port GPIOA
#define TAP13_UP_Pin GPIO_PIN_10
#define TAP13_UP_GPIO_Port GPIOC
#define TAP14_DOWN_Pin GPIO_PIN_11
#define TAP14_DOWN_GPIO_Port GPIOC
#define TAP14_UP_Pin GPIO_PIN_12
#define TAP14_UP_GPIO_Port GPIOC
#define TAP15_UP_Pin GPIO_PIN_2
#define TAP15_UP_GPIO_Port GPIOD
#define TAP4_UP_Pin GPIO_PIN_4
#define TAP4_UP_GPIO_Port GPIOB
#define TAP5_DOWN_Pin GPIO_PIN_5
#define TAP5_DOWN_GPIO_Port GPIOB
#define TAP5_UP_Pin GPIO_PIN_6
#define TAP5_UP_GPIO_Port GPIOB
#define TAP6_DOWN_Pin GPIO_PIN_7
#define TAP6_DOWN_GPIO_Port GPIOB
#define TAP6_UP_Pin GPIO_PIN_8
#define TAP6_UP_GPIO_Port GPIOB
#define TAP7_DOWN_Pin GPIO_PIN_9
#define TAP7_DOWN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
