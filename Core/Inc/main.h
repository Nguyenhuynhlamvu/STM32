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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Button_Pin GPIO_PIN_1
#define Button_GPIO_Port GPIOA
#define Button_EXTI_IRQn EXTI1_IRQn
#define pulseA_1_Pin GPIO_PIN_4
#define pulseA_1_GPIO_Port GPIOA
#define pulseA_1_EXTI_IRQn EXTI4_IRQn
#define pulseB_1_Pin GPIO_PIN_5
#define pulseB_1_GPIO_Port GPIOA
#define pulseA_2_Pin GPIO_PIN_6
#define pulseA_2_GPIO_Port GPIOA
#define pulseA_2_EXTI_IRQn EXTI9_5_IRQn
#define pulseB_2_Pin GPIO_PIN_7
#define pulseB_2_GPIO_Port GPIOA
#define PWM_3_Pin GPIO_PIN_0
#define PWM_3_GPIO_Port GPIOB
#define PWM_4_Pin GPIO_PIN_1
#define PWM_4_GPIO_Port GPIOB
#define ENA_1_Pin GPIO_PIN_11
#define ENA_1_GPIO_Port GPIOA
#define ENB_1_Pin GPIO_PIN_12
#define ENB_1_GPIO_Port GPIOA
#define ENA_2_Pin GPIO_PIN_15
#define ENA_2_GPIO_Port GPIOA
#define ENB_2_Pin GPIO_PIN_3
#define ENB_2_GPIO_Port GPIOB
#define PWM_1_Pin GPIO_PIN_4
#define PWM_1_GPIO_Port GPIOB
#define PWM_2_Pin GPIO_PIN_5
#define PWM_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
