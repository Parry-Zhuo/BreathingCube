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
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

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
#define ADC_BATTERY_MONITOR_Pin GPIO_PIN_1
#define ADC_BATTERY_MONITOR_GPIO_Port GPIOA
#define SPEAKER_Pin GPIO_PIN_3
#define SPEAKER_GPIO_Port GPIOA
#define Indication_light_Pin GPIO_PIN_4
#define Indication_light_GPIO_Port GPIOA
#define AddressableLED_Pin GPIO_PIN_6
#define AddressableLED_GPIO_Port GPIOA
#define BUTTON_4_Pin GPIO_PIN_7
#define BUTTON_4_GPIO_Port GPIOA
#define BUTTON_4_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON_1_Pin GPIO_PIN_0
#define BUTTON_1_GPIO_Port GPIOB
#define BUTTON_1_EXTI_IRQn EXTI0_IRQn
#define BUTTON_2_Pin GPIO_PIN_1
#define BUTTON_2_GPIO_Port GPIOB
#define BUTTON_2_EXTI_IRQn EXTI1_IRQn
#define BUTTON_3_Pin GPIO_PIN_15
#define BUTTON_3_GPIO_Port GPIOA
#define BUTTON_3_EXTI_IRQn EXTI15_10_IRQn
#define GY_521_INT_Pin GPIO_PIN_4
#define GY_521_INT_GPIO_Port GPIOB
#define GY_521_INT_EXTI_IRQn EXTI4_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
