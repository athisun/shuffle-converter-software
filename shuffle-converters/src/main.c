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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/

// clock setup moved from main.c
#include "clock.h"

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

extern void initialise_monitor_handles(void);

/* Private user code ---------------------------------------------------------*/

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


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  // semihosting init
  initialise_monitor_handles();
  
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();

  // perform an automatic ADC calibration to improve the conversion accuracy
  // has to be done before the adc is started
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler("error calbrating adc");
  }

  // disable switchers
  // ti switches are disabled when set/high (disable)
  // the other ones are enabled high
  HAL_GPIO_WritePin(DIS1_GPIO_Port, DIS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIS2_GPIO_Port, DIS2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIS3_GPIO_Port, DIS3_Pin, GPIO_PIN_SET);

  // set timers up
  // 50khz pwm freq
  // 2us pulse time
  // which timer
  // bottom channel
  // top channel
  // direction (0 = bottom to top, 1 = top to bottom)
  // timers_set(50000, 2, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, 0);

  /* Infinite loop */
  while (1)
  {
    // define an array where the adc values will be stored
    volatile uint32_t adc_values[7];
    // start the adc peripheral
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
      Error_Handler("error starting adc");
    }
    // read the adc peripheral a number of times for each "rank" aka channel setup
    for (uint8_t i = 0; i < 7; i++) {
      volatile HAL_StatusTypeDef result = HAL_ADC_PollForConversion(&hadc1, 1000);
      if (result != HAL_OK) {
        Error_Handler("error polling for conversion %d", i);
      }
      adc_values[i] = HAL_ADC_GetValue(&hadc1);
    }
    // stop the adc peripheral
    if (HAL_ADC_Stop(&hadc1) != HAL_OK) {
      Error_Handler("error stopping adc");
    }
    
    // calculate the analog refererence voltage from the vrefint channel
    volatile uint32_t vrefa = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc_values[6], ADC_RESOLUTION_12B);

    // calculate the internal temperature sensor from the internal channel
    volatile uint32_t temp = __HAL_ADC_CALC_TEMPERATURE(vrefa, adc_values[5], ADC_RESOLUTION_12B);
    
    // convert all the adc values to voltages using the analog reference voltage
    volatile uint32_t adc_voltages[5];
    for (uint8_t i = 0; i < 5; i++) {
      adc_voltages[i] = __HAL_ADC_CALC_DATA_TO_VOLTAGE(vrefa, adc_values[i], ADC_RESOLUTION_12B);
    }

    printf("adc reference voltage: %ld\n", vrefa);
    printf("internal temp: %ld\n", temp);
    
    for (uint8_t i = 0; i < 4; i++) {
      printf("VCC%d = %ld mV\n", 4-i+1, adc_voltages[i]);
    }

    printf("Current sense = %ld mV\n", adc_voltages[4]);
    

    HAL_Delay(1000);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);
  printf("\n");

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
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
