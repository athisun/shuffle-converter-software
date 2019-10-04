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
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/

// clock setup moved from main.c
#include "clock.h"

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

/* Private typedef -----------------------------------------------------------*/

// union type for converting floats to integers
union FloatConv {
  float f;
  uint32_t i;
};

/* Private define ------------------------------------------------------------*/

// id for can messages
#define CAN_ID_BASE 0x400
#define CAN_ID_CONFIG1 0x480
#define CAN_ID_CONFIG2 0x481
#define CAN_ID_ERROR 0x89

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// store the counts of the string cells
const uint8_t string_cell_counts[10][4] = {
    {6, 5, 10, 12},   // 0, A = 3
    {12, 10, 10},     // 1, B = 2
    {10, 10, 10, 10}, // 2, C = 3
    {10, 10, 10, 10}, // 3, D = 3
    {10, 10, 7, 14},  // 4, E = 3
    {14, 14, 14},     // 5, F = 2
    {12, 12},         // 6, G = 1
    {12, 12, 12, 12}, // 7, H = 3
    {12, 12, 12, 12}, // 8, I = 3
    {12, 14, 14, 14}, // 9, J = 3
};

const float adc_ratios[4] = {
    1.0 + 220.0 / 62.0,
    1.0 + 1300.0 / 160.0,
    1.0 + 1500.0 / 120.0,
    1.0 + 1300.0 / 75.0};

// sleep delay in ms between loops
const uint32_t iteration_period = 10; // 10 ms
// can update period in ms
const uint32_t can_update_period = 1000;
// pwm frequency (Hz)
const uint32_t pwm_freq = 100000;
// inductor params for limits
const float volt_usec_min = 0.1;
const float volt_usec_max = 40;
// maximum duty cycle for dcm mode
const float duty_max = 0.45;

// whether shuffling is actually enabled
volatile uint8_t shuffling_enabled = 0;
// which mode to shuffle in (0 = DCM, 1 = CCM)
volatile uint8_t shuffling_mode = 0;

/* Private function prototypes -----------------------------------------------*/

#ifdef DEBUG
extern void initialise_monitor_handles(void);
#endif

/* Private user code ---------------------------------------------------------*/

// simple debounce function for reading io pins
// only returns 1 if all read samples are 1
uint8_t debounce(GPIO_TypeDef *port, uint16_t pin)
{
  for (int i = 0; i < 8; i++)
  {
    if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET)
    {
      return 0;
    }
  }
  return 1;
}

// sends a can message with given id, length and data
// returns the tx mailbox id on success, -1 on failure
int32_t can_send(uint16_t id, uint8_t len, uint8_t data[])
{
  // limit message data length to 8
  if (len > 8)
  {
    len = 8;
  }

  CAN_TxHeaderTypeDef header;
  header.StdId = (CAN_ID_BASE + id);
  header.IDE = CAN_ID_STD;   // standard id
  header.RTR = CAN_RTR_DATA; // data frame
  header.DLC = len;          // size of data in bytes
  header.TransmitGlobalTime = DISABLE;

  uint32_t count = 0;
  uint8_t aborted = 0;
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
  {
    count++;
    HAL_Delay(1);
    if (count > 5)
    {
      count = 0;
      // if mailbox requests have already been aborted, return failure
      if (aborted)
      {
        return -1;
      }
      // abort all pending tx mailbox, the idea being only the latest can messages should be queued to send
      // TODO: find a define of the count of mailboxes somewhere
      //       seems to be 2 so hardcode?
      aborted = 1;
      for (uint8_t i = 0; i <= 2; i++)
      {
        HAL_CAN_AbortTxRequest(&hcan1, i);
      }
    }
  }

  uint32_t mailbox;
  if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) != HAL_OK)
  {
    // dont enter sos mode here, instead transparently ignore error
    // Error_Handler("error sending can msg");
    return -1;
  }

  return mailbox;
}

// send a single byte can message
int8_t can_send_id(uint8_t id, uint8_t dip)
{
  return can_send(id, 1, &dip);
}

// send a two byte can message
// first byte is the dip switch configuration
// data byte is the dip switch configuration
int8_t can_send_u8(uint8_t id, uint8_t dip, uint8_t data)
{
  uint8_t b[2] = {dip, data};
  return can_send(id, sizeof(b), b);
}

// send a three byte can message
// first byte is the dip switch configuration
// second two are the u16 data
int8_t can_send_u16(uint8_t id, uint8_t dip, uint16_t data)
{
  uint8_t b[3] = {dip};
  b[1] = (data >> 8) & 0xff;
  b[2] = data & 0xff;
  return can_send(id, sizeof(b), b);
}

// send a five byte can message
// first byte is the dip switch configuration
// next four are the u32 data
int8_t can_send_u32(uint8_t id, uint8_t dip, uint32_t data)
{
  uint8_t b[5] = {dip};
  b[1] = (data >> 24) & 0xff;
  b[2] = (data >> 16) & 0xff;
  b[3] = (data >> 8) & 0xff;
  b[4] = data & 0xff;
  return can_send(id, sizeof(b), b);
}

// send an eight byte can message
// first byte is the dip switch configuration
// next 7 are the u64 data
// NOTE: top byte is essentially masked from u64 data (ie, not included)
int8_t can_send_u64(uint8_t id, uint8_t dip, uint64_t data)
{
  uint8_t b[8] = {dip};
  b[1] = (data >> 48) & 0xff;
  b[2] = (data >> 40) & 0xff;
  b[3] = (data >> 32) & 0xff;
  b[4] = (data >> 24) & 0xff;
  b[5] = (data >> 16) & 0xff;
  b[6] = (data >> 8) & 0xff;
  b[7] = data & 0xff;
  return can_send(id, sizeof(b), b);
}

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
void pwm_configure_channel(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t mode, uint32_t polarity, uint32_t pulse)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = mode;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = polarity;
  // sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  // sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel) != HAL_OK)
  {
    Error_Handler("error configuring timer %d channel %d pwm (mode: %ld, pulse: %ld)", get_timer_num(htim), Channel, mode, pulse);
  }
}

void disable_timer(TIM_HandleTypeDef *htim, const uint32_t channel1, const uint32_t channel2,
                   GPIO_TypeDef *dis_port, const uint32_t dis_pin)
{
  // disable gate driver
  HAL_GPIO_WritePin(dis_port, dis_pin, GPIO_PIN_SET);

  // stop the timers before making changes to pwm
  if (HAL_TIM_PWM_Stop(htim, channel1) != HAL_OK)
  {
    uint8_t timernum = get_timer_num(htim);
    Error_Handler("error stopping timer %d channel %d", timernum, channel1);
  }
  if (HAL_TIM_PWM_Stop(htim, channel2) != HAL_OK)
  {
    uint8_t timernum = get_timer_num(htim);
    Error_Handler("error stopping timer %d channel %d", timernum, channel2);
  }
}

void calc_timer(uint32_t f_tim, uint32_t f_pwm, float dc, uint32_t *psc, uint32_t *arr, uint32_t *ccrx)
{
  // the PWM resolution gives the number of possible duty cycle values and indicates how fine the control on the PWM signal will be
  // The resolution, expressed in number of duty cycle steps, is simply equal to the ratio between the timer clock frequency
  // and the PWM frequency, the whole minus 1.
  // TODO: error if the resolution is too low? whats too low?
  // float res = 1.0*f_tim/f_pwm;

  // start with a prescaler value of 0
  *psc = 0;
  // calculate the autoreload register
  *arr = f_tim / f_pwm;

  // assume 16 bit timer, adjust prescaler until arr is within 16 bit
  // could probably adjust this for the 32 bit timer but keep it consistent
  while ((*arr) > 0xFFFF)
  {
    (*psc)++;
    *arr = (f_tim / ((*psc) + 1)) / f_pwm;
  }

  // calculate counter reload register value (pulse count)
  *ccrx = (dc * ((*arr) + 1)) - 1;
}

void start_shuffling(TIM_HandleTypeDef *htim, const uint32_t channel1, const uint32_t channel2,
                     GPIO_TypeDef *dis_port, const uint32_t dis_pin,
                     uint32_t psc, uint32_t arr, uint32_t ccrx,
                     float direction)
{

  // reinititialise timer with new prescaler and period value
  htim->Init.Prescaler = psc;
  htim->Init.Period = arr;
  if (HAL_TIM_PWM_Init(htim) != HAL_OK)
  {
    uint8_t timernum = get_timer_num(htim);
    Error_Handler("error initialising timer %d pwm (psc: %ld, arr: %ld)", timernum, psc, arr);
  }

  // configure each timer pwm mode and pulse
  if (direction > 0)
  {
    // if the direction is positive (up the string), switch the bottom gate first
    pwm_configure_channel(htim, channel1, TIM_OCMODE_PWM1, TIM_OCPOLARITY_HIGH, ccrx);
    pwm_configure_channel(htim, channel2, TIM_OCMODE_PWM2, TIM_OCPOLARITY_HIGH, arr - ccrx);
  }
  else
  {
    // if the direction is negative (down the string), switch the top gate first
    pwm_configure_channel(htim, channel1, TIM_OCMODE_PWM1, TIM_OCPOLARITY_LOW, arr - ccrx);
    pwm_configure_channel(htim, channel2, TIM_OCMODE_PWM2, TIM_OCPOLARITY_LOW, ccrx);
  }

  // start pwm timers
  if (HAL_TIM_PWM_Start(htim, channel1) != HAL_OK)
  {
    uint8_t timernum = get_timer_num(htim);
    Error_Handler("error starting from timer %d channel %d", timernum, channel1);
  }
  if (HAL_TIM_PWM_Start(htim, channel2) != HAL_OK)
  {
    uint8_t timernum = get_timer_num(htim);
    Error_Handler("error starting to timer %d channel %d", timernum, channel2);
  }

  // enable gate driver
  HAL_GPIO_WritePin(dis_port, dis_pin, GPIO_PIN_RESET);
}

// open loop shuffle
void do_shuffle_open(TIM_HandleTypeDef *htim, const uint32_t channel1, const uint32_t channel2,
                     GPIO_TypeDef *dis_port, const uint32_t dis_pin,
                     const uint8_t cell_count1, const uint8_t cell_count2,
                     const uint32_t f_tim, const uint32_t f_pwm,
                     float *duty_cycle, float *direction)
{
  // if no shuffling to be performed on this input, return after timers and gate drivers are disabled/off
  if (cell_count1 == 0 || cell_count2 == 0)
  {
    disable_timer(htim, channel1, channel2, dis_port, dis_pin);
    return;
  }

  // determine direction & dc based on cell counts
  if (cell_count1 < cell_count2)
  {
    *direction = -1;
    *duty_cycle = (*direction) * cell_count2 / (cell_count1 + cell_count2);
  }
  else
  {
    *direction = 1;
    *duty_cycle = (*direction) * cell_count2 / (cell_count1 + cell_count2);
  }

  // calculate timer parameters
  uint32_t psc, arr, ccrx;
  calc_timer(f_tim, f_pwm, *duty_cycle, &psc, &arr, &ccrx);

  // start the timers shuffling
  start_shuffling(htim, channel1, channel2,
                  dis_port, dis_pin,
                  psc, arr, ccrx,
                  *direction);
}

// closed loop shuffle
// starts the shuffling process on a given timer between two channels and voltage readings for those channels
// timer channel 1, vin1 (string voltage), and cell_count1 (number of cells in string) are the low side
// timer channel 2, vin2 (string voltage), and cell_count2 (number of cells in string) are the high side
// timer and pwm frequency are in Hz
// duty cycle and direction are pointers and will be modified to reflect the updated values
void do_shuffle_closed(TIM_HandleTypeDef *htim, const uint32_t channel1, const uint32_t channel2,
                       GPIO_TypeDef *dis_port, const uint32_t dis_pin,
                       const uint32_t vin1, const uint32_t vin2, const uint8_t cell_count1, const uint8_t cell_count2,
                       const uint32_t f_tim, const uint32_t f_pwm,
                       float *duty_cycle, float *direction,
                       const float v_usec_min, const float v_usec_max)
{
  // if no shuffling to be performed on this input, return after timers and gate drivers are disabled/off
  if (cell_count1 == 0 || cell_count2 == 0)
  {
    disable_timer(htim, channel1, channel2, dis_port, dis_pin);
    return;
  }

  // normalise the string -> cell voltages
  float vin1_norm = vin1 / (float)cell_count1;
  float vin2_norm = vin2 / (float)cell_count2;

  // normalise the cell voltages and calculate the error
  float err = vin2_norm - vin1_norm;

  // adjust the duty cycle based on the gain and error
  float D = (*duty_cycle) * (*direction) + 0.001 * err;

  // limits: -duty_max <= duty cycle <= duty_max
  if (D < -duty_max)
  {
    D = -duty_max;
  }
  else if (D > duty_max)
  {
    D = duty_max;
  }

  // copy the sign from the duty cycle to store direction of switching
  *direction = copysignf(1.0, D);

  // copy the absolute value from the duty cycle and store
  *duty_cycle = fabsf(D);

  // calculate inductor properties to determine timings
  float period = (1 / (float)f_tim);
  float period_us = period * pow(10, 6);

  // calculate timer parameters
  uint32_t psc, arr, ccrx;
  calc_timer(f_tim, f_pwm, *duty_cycle, &psc, &arr, &ccrx);

  // check the inductor volt.second calcs
  float on_time_us = period_us * ccrx;

  if (on_time_us < v_usec_min)
  {
    // don't shuffle
    disable_timer(htim, channel1, channel2, dis_port, dis_pin);
    return;
  }
  else if (on_time_us > v_usec_max)
  {
    ccrx = period_us / v_usec_max;
  }

  // start the timers shuffling
  start_shuffling(htim, channel1, channel2,
                  dis_port, dis_pin,
                  psc, arr, ccrx,
                  *direction);
}

// measure adc values
// note: string_voltages[n] decays to a pointer (ie: uint32_t *string_voltages), not the best way to do this
// just an indiciation to user as to the size of the array that should be passed in
void measure_adc(uint32_t string_voltages[4], uint32_t *vcurrentsense, uint32_t *vrefa, uint32_t *temp)
{
  // define an array where the adc values will be stored
  uint32_t adc_values[7];
  // array FOR converted adc voltages
  uint32_t adc_voltages[5];
  // start the adc peripheral
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    Error_Handler("error starting adc");
  }
  // read the adc peripheral a number of times for each "rank" aka channel setup
  for (uint8_t i = 0; i < 7; i++)
  {
    HAL_StatusTypeDef result = HAL_ADC_PollForConversion(&hadc1, 1000);
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
  *vrefa = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc_values[6], ADC_RESOLUTION_12B);

  // calculate the internal temperature sensor from the internal channel
  *temp = __HAL_ADC_CALC_TEMPERATURE(*vrefa, adc_values[5], ADC_RESOLUTION_12B);

  // convert all the adc values to voltages using the analog reference voltage
  for (uint8_t i = 0; i < 5; i++)
  {
    adc_voltages[i] = __HAL_ADC_CALC_DATA_TO_VOLTAGE(*vrefa, adc_values[i], ADC_RESOLUTION_12B);
  }

  // convert using the adc ratio
  for (uint8_t i = 0; i < 4; i++)
  {
    // TODO: put in resistor values
    string_voltages[i] = adc_voltages[i] * adc_ratios[i]; // (15.0 * (i + 1.0)) / 3.3;
  }
}

// read dip switches
uint8_t read_dip()
{
  // use ! because internal pullup, dip pin pulls down to gnd
  uint8_t dip1 = !debounce(DIP1_GPIO_Port, DIP1_Pin);
  uint8_t dip2 = !debounce(DIP2_GPIO_Port, DIP2_Pin);
  uint8_t dip3 = !debounce(DIP3_GPIO_Port, DIP3_Pin);
  uint8_t dip4 = !debounce(DIP4_GPIO_Port, DIP4_Pin);
  uint8_t dip = ((dip4 & 0x1) << 3) | ((dip3 & 0x1 << 2)) | ((dip2 & 0x1) << 1) | (dip1 & 0x1);
  return dip & 0xf;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8] = {};
  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &header, data);

  uint8_t old_enabled = shuffling_enabled;

  switch (header.StdId)
  {
  case CAN_ID_CONFIG1:
    // byte 0 = dip
    // byte 1 = enable
    // byte 2 = mode
    if (header.DLC != 3)
    {
      return;
    }
    uint8_t dip = read_dip();
    if (data[0] != dip)
    {
      // packet not fur us
      return;
    }
    shuffling_enabled = data[1];
    shuffling_mode = data[2];

    break;

  case CAN_ID_CONFIG2:
    // byte 0 = enable
    if (header.DLC != 1)
    {
      return;
    }
    shuffling_enabled = data[0];
    break;

  default:
    // just ignore it
    return;
  }

  if (old_enabled == 1 && shuffling_enabled == 0)
  {
    // if the shuffling has been disabled, specifically turn things off
    disable_timer(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, DIS1_GPIO_Port, DIS1_Pin);
    disable_timer(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, DIS2_GPIO_Port, DIS2_Pin);
    disable_timer(&htim15, TIM_CHANNEL_1, TIM_CHANNEL_2, DIS3_GPIO_Port, DIS3_Pin);
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

  /*
  // grab the reset flags
  volatile const uint8_t rst_bor = __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST);
  volatile const uint8_t rst_obl = __HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST);
  volatile const uint8_t rst_pin = __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST);
  volatile const uint8_t rst_fwr = __HAL_RCC_GET_FLAG(RCC_FLAG_FWRST);
  volatile const uint8_t rst_sft = __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST);
  volatile const uint8_t rst_iwdg = __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST);
  volatile const uint8_t rst_wwdg = __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST);
  volatile const uint8_t rst_lpwr = __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST);

  __HAL_RCC_CLEAR_RESET_FLAGS();
  */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_IWDG_Init();

  // perform an automatic ADC calibration to improve the conversion accuracy
  // has to be done before the adc is started
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler("error calbrating adc");
  }

  // disable gate drivers
  HAL_GPIO_WritePin(DIS1_GPIO_Port, DIS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIS2_GPIO_Port, DIS2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIS3_GPIO_Port, DIS3_Pin, GPIO_PIN_SET);

  // set up a can filter for ids
  // http://schulz-m.github.io/2017/03/23/stm32-can-id-filter/
  // http://www.cse.dmu.ac.uk/~eg/tele/CanbusIDandMask.html
  CAN_FilterTypeDef sFilterConfig = {};
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh = CAN_ID_CONFIG1;
  sFilterConfig.FilterIdLow = CAN_ID_CONFIG2;
  sFilterConfig.FilterMaskIdHigh = 0xFFFFFFFF;
  sFilterConfig.FilterMaskIdLow = 0xFFFFFFFF;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler("error setting can filter");
  }

  // start can
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler("error starting can");
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  { /* Notification Error */
    Error_Handler("error activating can notifications");
  }

  // stores the time in ms that the last can messages was sent
  uint32_t can_last_update = HAL_GetTick();

  // structures to store shuffle converter direction and duty cycles
  struct
  {
    union FloatConv direction;
    union FloatConv duty_cycle;
  } shuffle_converter1 = {0}, shuffle_converter2 = {0}, shuffle_converter3 = {0};

  // mcu temp and analog ref voltage
  uint32_t vrefa = 0, temp = 0, vcurrentsense = 0;
  // string cumulative voltages
  uint32_t VCC[4] = {};

  /* Infinite loop */
  while (1)
  {
    // referesh the watchdog every loop
    HAL_IWDG_Refresh(&hiwdg);

    // sleep for the configured iteration period
    HAL_Delay(iteration_period);

    // measure adc values
    measure_adc(VCC, &vcurrentsense, &vrefa, &temp);

    // read dip switch
    uint8_t dip = read_dip();

    // convert the cumulative string voltages to individual string voltages
    uint32_t VCC2_1 = VCC[0];
    uint32_t VCC3_2 = VCC[1] - VCC[0];
    uint32_t VCC4_3 = VCC[2] - VCC[1];
    uint32_t VCC5_4 = VCC[3] - VCC[2];

    // send a can packet once every update period
    if (HAL_GetTick() - can_last_update > can_update_period)
    {
      can_last_update = HAL_GetTick();

      // send shuffling status
      can_send_u16(0, dip, (shuffling_enabled << 8) | (shuffling_mode & 0xff));

      // send mcu temp and adc vref
      can_send_u32(1, dip, (temp << 16) | (vrefa & 0xffff));

      // send all read string voltages, assume string voltages aren't greater than 16 bit
      // ie, no voltage over 2^16=65535mV or 65.5V
      can_send_u32(2, dip, (VCC2_1 << 16) | (VCC3_2 & 0xffff));
      can_send_u32(3, dip, (VCC4_3 << 16) | (VCC5_4 & 0xffff));

      // and converted shuffle duty cycles + directions
      // inlcude both the sign for direction and duty cycle
      if (shuffling_enabled)
      {
        union FloatConv dc;
        dc.f = shuffle_converter1.duty_cycle.f * (shuffle_converter1.direction.f == 0 ? 1 : shuffle_converter1.direction.f);
        can_send_u32(4, dip, dc.i);
        dc.f = shuffle_converter2.duty_cycle.f * (shuffle_converter2.direction.f == 0 ? 1 : shuffle_converter2.direction.f);
        can_send_u32(5, dip, dc.i);
        dc.f = shuffle_converter3.duty_cycle.f * (shuffle_converter3.direction.f == 0 ? 1 : shuffle_converter3.direction.f);
        can_send_u32(6, dip, dc.i);
      }

      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    // only shuffle if enabled by config
    if (shuffling_enabled == 0)
    {
      continue;
    }

    switch (shuffling_mode)
    {
    case 0:
      // closed loop shuffling
      do_shuffle_closed(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2,
                        DIS1_GPIO_Port, DIS1_Pin,
                        VCC2_1, VCC3_2, string_cell_counts[dip][0], string_cell_counts[dip][1],
                        HAL_RCC_GetPCLK1Freq(), pwm_freq,
                        &shuffle_converter1.duty_cycle.f, &shuffle_converter1.direction.f,
                        volt_usec_min, volt_usec_max);

      do_shuffle_closed(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2,
                        DIS2_GPIO_Port, DIS2_Pin,
                        VCC3_2, VCC4_3, string_cell_counts[dip][1], string_cell_counts[dip][2],
                        HAL_RCC_GetPCLK2Freq(), pwm_freq,
                        &shuffle_converter2.duty_cycle.f, &shuffle_converter2.direction.f,
                        volt_usec_min, volt_usec_max);

      do_shuffle_closed(&htim15, TIM_CHANNEL_1, TIM_CHANNEL_2,
                        DIS3_GPIO_Port, DIS3_Pin,
                        VCC4_3, VCC5_4, string_cell_counts[dip][2], string_cell_counts[dip][3],
                        HAL_RCC_GetPCLK1Freq(), pwm_freq,
                        &shuffle_converter3.duty_cycle.f, &shuffle_converter3.direction.f,
                        volt_usec_min, volt_usec_max);
      break;

    case 1:
      // open loop shuffling
      do_shuffle_open(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2,
                      DIS1_GPIO_Port, DIS1_Pin,
                      string_cell_counts[dip][0], string_cell_counts[dip][1],
                      HAL_RCC_GetPCLK1Freq(), pwm_freq,
                      &shuffle_converter1.duty_cycle.f, &shuffle_converter1.direction.f);

      do_shuffle_open(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2,
                      DIS2_GPIO_Port, DIS2_Pin,
                      string_cell_counts[dip][1], string_cell_counts[dip][2],
                      HAL_RCC_GetPCLK2Freq(), pwm_freq,
                      &shuffle_converter2.duty_cycle.f, &shuffle_converter2.direction.f);

      do_shuffle_open(&htim15, TIM_CHANNEL_1, TIM_CHANNEL_2,
                      DIS3_GPIO_Port, DIS3_Pin,
                      string_cell_counts[dip][2], string_cell_counts[dip][3],
                      HAL_RCC_GetPCLK1Freq(), pwm_freq,
                      &shuffle_converter3.duty_cycle.f, &shuffle_converter3.direction.f);
      break;

    default:
      Error_Handler("unknown shuffle mode: %d", shuffling_mode);
      break;
    }
  }
}

void blinky(uint8_t n, uint32_t delay1, uint32_t delay2, uint32_t delay3)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  for (uint8_t i = 0; i < n; i++)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(delay1);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(delay2);
  }
  HAL_Delay(delay3);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(const char *format, ...)
{

  // disable all the gate drivers, in an attempt to set to a "safe" state
  disable_timer(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, DIS1_GPIO_Port, DIS1_Pin);
  disable_timer(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, DIS2_GPIO_Port, DIS2_Pin);
  disable_timer(&htim15, TIM_CHANNEL_1, TIM_CHANNEL_2, DIS3_GPIO_Port, DIS3_Pin);

  // send error to can bus
  can_send_id(CAN_ID_ERROR, read_dip());

  /*
  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);
  */
  // printf("\n");

  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
    // blink sos
    blinky(3, 100, 100, 200);
    blinky(3, 300, 100, 200);
    blinky(3, 100, 100, 1000);
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
  Error_Handler("Wrong parameters value: file %s on line %d\r\n", file, line)
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
