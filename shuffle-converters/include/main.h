/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM5_Pin GPIO_PIN_0
#define PWM5_GPIO_Port GPIOA
#define PWM6_Pin GPIO_PIN_1
#define PWM6_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_2
#define PWM3_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_3
#define PWM4_GPIO_Port GPIOA
#define DIS2_Pin GPIO_PIN_4
#define DIS2_GPIO_Port GPIOA
#define ADC1_Pin GPIO_PIN_5
#define ADC1_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_6
#define ADC2_GPIO_Port GPIOA
#define ADC3_Pin GPIO_PIN_7
#define ADC3_GPIO_Port GPIOA
#define ADC4_Pin GPIO_PIN_0
#define ADC4_GPIO_Port GPIOB
#define ADC5_Pin GPIO_PIN_1
#define ADC5_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_8
#define PWM2_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOA
#define DIS1_Pin GPIO_PIN_10
#define DIS1_GPIO_Port GPIOA
#define DIP4_Pin GPIO_PIN_15
#define DIP4_GPIO_Port GPIOA
#define DIP3_Pin GPIO_PIN_3
#define DIP3_GPIO_Port GPIOB
#define DIP2_Pin GPIO_PIN_4
#define DIP2_GPIO_Port GPIOB
#define DIP1_Pin GPIO_PIN_5
#define DIP1_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOB
#define DIS3_Pin GPIO_PIN_7
#define DIS3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
