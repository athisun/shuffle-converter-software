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

#ifdef DEBUG
extern void initialise_monitor_handles(void);
#endif

/* Private user code ---------------------------------------------------------*/

// returns the timer number from the pointer based on the instance
uint8_t get_timer_num(TIM_HandleTypeDef *htim)
{
  uint8_t timernum = 0;
  if (htim->Instance == TIM1)
  {
    timernum = 1;
  }
  else if (htim->Instance == TIM2)
  {
    timernum = 2;
  }
  else if (htim->Instance == TIM15)
  {
    timernum = 15;
  }
  return timernum;
}

// configure a timer pwm channel mode and pulse
void pwm_configure_channel(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t mode, uint32_t pulse)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = mode;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel) != HAL_OK)
  {
    Error_Handler("error configuring timer %d channel %d pwm (mode: %ld, pulse: %ld)", get_timer_num(htim), Channel, mode, pulse);
  }
}

// configures a pwm timer and channels based on freq, pulse width
// this stops, configures, then starts the pwm
// formulas from http://www.emcu.eu/wp-content/uploads/2016/11/en.STM32L4_WDG_TIMERS_GPTIM.pdf
void pwm_configure_timer(uint32_t pwm_frequency, uint32_t pulse_width_us, TIM_HandleTypeDef *htim, uint32_t channel_from, uint32_t channel_to)
{

  float period = 1.0 / pwm_frequency;
  float pulse_width = pulse_width_us * pow(10, -6);
  float duty = (float)((int)((pulse_width / period) * 100 + 0.5) / 100.0);

  if (duty <= 0 || duty >= 1)
  {
    return;
  }

  uint8_t timernum = get_timer_num(htim);

  // stop the timers before making changes to pwm
  if (HAL_TIM_PWM_Stop(htim, channel_from) != HAL_OK)
  {
    Error_Handler("error stopping from timer %d channel %d", timernum, channel_from);
  }
  if (HAL_TIM_PWM_Stop(htim, channel_to) != HAL_OK)
  {
    Error_Handler("error stopping to timer %d channel %d", timernum, channel_to);
  }

  // get the frequency of the right timer clock based on the timer number
  // TODO: is there a nicer way to do this?
  uint32_t ftim = 0;
  switch (timernum)
  {
  case 1:
  case 15:
    ftim = HAL_RCC_GetPCLK1Freq();
    break;
  case 2:
    ftim = HAL_RCC_GetPCLK2Freq();
    break;
  default:
    Error_Handler("unkown timer");
    break;
  }

  // the PWM resolution gives the number of possible duty cycle values and indicates how fine the control on the PWM signal will be
  // The resolution, expressed in number of duty cycle steps, is simply equal to the ratio between the timer clock frequency
  // and the PWM frequency, the whole minus 1.
  // TODO: error if the resolution is too low? whats too low?
  // float res = 1.0*ftim/pwm_frequency;

  // start with a prescaler value of 0
  uint32_t psc = 0;
  // calculate the autoreload register
  uint32_t arr = (ftim / (psc + 1)) / pwm_frequency;

  // assume 16 bit timer, adjust prescaler until arr is within 16 bit
  // could probably adjust this for the 32 bit timer but keep it consistent
  while (arr > 0xFFFF)
  {
    psc++;
    arr = (ftim / (psc + 1)) / pwm_frequency;
  }

  // reinititialise timer with new prescaler and period value
  htim->Init.Prescaler = psc;
  htim->Init.Period = arr;
  if (HAL_TIM_PWM_Init(htim) != HAL_OK)
  {
    Error_Handler("error initialising timer %d pwm (psc: %ld, arr: %ld)", timernum, psc, arr);
  }

  // calculate counter reload register value (pulse count)
  uint32_t ccrx = (duty * (arr + 1)) - 1;

  // configure each timer pwm mode and pulse
  pwm_configure_channel(htim, channel_from, (duty < 50 ? TIM_OCMODE_PWM1 : TIM_OCMODE_PWM2), ccrx);
  pwm_configure_channel(htim, channel_to, (duty < 50 ? TIM_OCMODE_PWM2 : TIM_OCMODE_PWM1), arr - ccrx);

  // start pwm timers
  if (HAL_TIM_PWM_Start(htim, channel_from) != HAL_OK)
  {
    Error_Handler("error starting from timer %d channel %d", timernum, channel_from);
  }
  if (HAL_TIM_PWM_Start(htim, channel_to) != HAL_OK)
  {
    Error_Handler("error starting to timer %d channel %d", timernum, channel_to);
  }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
#ifdef DEBUG
  // semihosting init
  initialise_monitor_handles();
#endif

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
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler("error calbrating adc");
  }

  // disable switchers
  // ti switches are disabled when set/high (disable pin)
  // the other new ones are enabled high (enable pin)
  HAL_GPIO_WritePin(DIS1_GPIO_Port, DIS1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIS2_GPIO_Port, DIS2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIS3_GPIO_Port, DIS3_Pin, GPIO_PIN_RESET);

  // how to set a timer up
  // 50khz pwm freq
  // 2us pulse time
  // which timer
  // from channel
  // to channel
  /*
  pwm_configure_timer(50000, 2, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2);
  pwm_configure_timer(50000, 2, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2);
  pwm_configure_timer(50000, 2, &htim15, TIM_CHANNEL_1, TIM_CHANNEL_2);
  */

  /* Infinite loop */
  while (1)
  {
    // define an array where the adc values will be stored
    volatile uint32_t adc_values[7];
    // start the adc peripheral
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
      Error_Handler("error starting adc");
    }
    // read the adc peripheral a number of times for each "rank" aka channel setup
    for (uint8_t i = 0; i < 7; i++)
    {
      volatile HAL_StatusTypeDef result = HAL_ADC_PollForConversion(&hadc1, 1000);
      if (result != HAL_OK)
      {
        Error_Handler("error polling for conversion %d", i);
      }
      adc_values[i] = HAL_ADC_GetValue(&hadc1);
    }
    // stop the adc peripheral
    if (HAL_ADC_Stop(&hadc1) != HAL_OK)
    {
      Error_Handler("error stopping adc");
    }

    // calculate the analog refererence voltage from the vrefint channel
    volatile uint32_t vrefa = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc_values[6], ADC_RESOLUTION_12B);

    // calculate the internal temperature sensor from the internal channel
    volatile uint32_t temp = __HAL_ADC_CALC_TEMPERATURE(vrefa, adc_values[5], ADC_RESOLUTION_12B);

    // convert all the adc values to voltages using the analog reference voltage
    volatile uint32_t adc_voltages[5];
    for (uint8_t i = 0; i < 5; i++)
    {
      adc_voltages[i] = __HAL_ADC_CALC_DATA_TO_VOLTAGE(vrefa, adc_values[i], ADC_RESOLUTION_12B);
    }

    printf("adc reference voltage: %ld\n", vrefa);
    printf("internal temp: %ld\n", temp);

    for (uint8_t i = 0; i < 4; i++)
    {
      printf("VCC%d = %ld mV\n", 4 - i + 1, adc_voltages[i]);
    }

    printf("Current sense = %ld mV\n", adc_voltages[4]);

    printf("dip 1: %d\n", HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin));
    printf("dip 2: %d\n", HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin));
    printf("dip 3: %d\n", HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin));
    printf("dip 4: %d\n", HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin));

    // TODO: convert voltages from adc range to actual range using voltage divider calcs? or just work with adc reading voltages

    // TODO: adjust pwm duty cycle or on time?
    // should probably be on time due to the ability to calculate current through inductor and limit?

    HAL_Delay(1000);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(const char *format, ...)
{
  // TODO: this should disable all the gate drivers, in an attempt to set to a "safe" state

  // TODO: check if can is initialised a this point, and send the error message to can bus

  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);
  printf("\n");

  /* User can add his own implementation to report the HAL error return state */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  while (1)
  {
    for (int i = 0; i < 3; i++)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
    }
    HAL_Delay(200);
    for (int i = 0; i < 3; i++)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(300);
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
    }
    HAL_Delay(200);
    for (int i = 0; i < 3; i++)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
    }
    HAL_Delay(1000);
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number, */
  printf("Wrong parameters value: file %s on line %d\r\n", file, line)
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
