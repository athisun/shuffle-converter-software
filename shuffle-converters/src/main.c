/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// clock setup moved from main.c
#include "clock.h"

#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// duty cycle is in percent, 0.0 - 1.0


void timer_configure_pwm(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t fpwm, double duty, uint32_t mode) {

  // formulas from http://www.emcu.eu/wp-content/uploads/2016/11/en.STM32L4_WDG_TIMERS_GPTIM.pdf

  uint32_t ftim = HAL_RCC_GetSysClockFreq();
  uint32_t psc = 0;
  uint32_t arr = (ftim/(psc+1))/fpwm;

  while(arr > 0xFFFF) {
    psc++;
    arr = (ftim/(psc+1))/fpwm;
  }

  uint32_t ccrx = (duty * (arr+1)) - 1;

  volatile double res = 1.0*ftim/fpwm;
  
  // stop generation of pwm
  volatile HAL_StatusTypeDef out;
  out = HAL_TIM_PWM_Stop(htim, Channel);
  TIM_OC_InitTypeDef sConfigOC = {0};
  // set the period duration
  htim->Init.Prescaler = psc;
  htim->Init.Period = arr;
  // reinititialise with new period value
  out = HAL_TIM_PWM_Init(htim);
  // set the pulse duration
  sConfigOC.OCMode = mode;
  sConfigOC.Pulse = ccrx;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  out = HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel);
  // start pwm generation
  out = HAL_TIM_PWM_Start(htim, Channel);
}

void timers_set(uint32_t pwm_frequency, uint32_t pulse_width_us, TIM_HandleTypeDef *htim, uint32_t channel_bottom, uint32_t channel_top, uint32_t direction) {

  volatile double period = 1.0 / pwm_frequency;
  volatile double pulse_width = pulse_width_us * pow(10, -6);
  double duty = (double)((int)((pulse_width/period) * 100 + 0.5) / 100.0);

  if (duty <= 0 || duty >= 1) {
    return;
  }

  // start timer 1 channel 1
  timer_configure_pwm(htim, (direction == 1 ? channel_top : channel_bottom), pwm_frequency, duty, (duty < 50 ? TIM_OCMODE_PWM1 : TIM_OCMODE_PWM2));
  timer_configure_pwm(htim, (direction == 1 ? channel_bottom : channel_top), pwm_frequency, 1.0-duty, (duty < 50 ? TIM_OCMODE_PWM2 : TIM_OCMODE_PWM1));
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  // disable switchers
  // HAL_GPIO_WritePin(DIS1_GPIO_Port, DIS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIS2_GPIO_Port, DIS2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIS3_GPIO_Port, DIS3_Pin, GPIO_PIN_SET);

  // set timers up
  // 50khz pwm freq
  // 2us pulse time
  // which timer
  // bottom channel
  // top channel
  // direction (0 = bottom to top, 1 = top to bottom)
  timers_set(50000, 2, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, 0);

  // adc example, read channnel 10 (adc1 on schematic)
  /*
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_ADC_Start(&hadc1);
  volatile HAL_StatusTypeDef result = HAL_ADC_PollForConversion(&hadc1, 1000);
  volatile uint32_t adc = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, GPIO_PIN_RESET);
  while(1){
    for( int i = 0; i<3; i++){
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
    }
    HAL_Delay(200);
    for( int i = 0; i<3; i++){
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(300);
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
    }
    HAL_Delay(200);
    for( int i = 0; i<3; i++){
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
    }
    HAL_Delay(1000);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
