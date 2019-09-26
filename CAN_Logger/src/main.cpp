/*
  Listfiles

  This example shows how print out the files in a
  directory on a SD card

  The circuit:
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

  created   Nov 2010
  by David A. Mellis
  modified 9 Apr 2012
  by Tom Igoe
  modified 2 Feb 2014
  by Scott Fitzgerald

  This example code is in the public domain.

*/
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <can.h>

File root;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
/* USER CODE END PV */

/* USER CODE BEGIN 4 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
  HAL_GPIO_TogglePin(GPIOB, LED_BUILTIN);
  Serial.println("NEW CAN MESSAGE !!!!!!!!!!!!!!!!!!!!!!");

  Serial.print("StdId: ");
  Serial.println(RxHeader.StdId);
  Serial.print("ExtId: ");
  Serial.println(RxHeader.ExtId);
  Serial.print("IDE: ");
  Serial.println(RxHeader.IDE);
  Serial.print("RTR: ");
  Serial.println(RxHeader.RTR);
  Serial.print("DLC: ");
  Serial.println(RxHeader.DLC);
  Serial.print("Timestamp: ");
  Serial.println(RxHeader.Timestamp);
  Serial.print("FilterMatchIndex: ");
  Serial.println(RxHeader.FilterMatchIndex);

  Serial.print("Data:");
  for (int i = 0; i < RxHeader.DLC; i++)
  {
    Serial.print(RxData[i]);
    Serial.print(" ");
  }
  Serial.println("");
}
/* USER CODE END 4 */

void printDirectory(File dir, int numTabs)
{
  while (true)
  {

    File entry = dir.openNextFile();
    if (!entry)
    {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++)
    {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory())
    {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    }
    else
    {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  while (!SD.begin(3))
  {
    Serial.println("initialization failed!");
    delay(50);
  }
  Serial.println("initialization done.");

  root = SD.open("/");

  printDirectory(root, 0);

  Serial.println("done!");

  if (MX_CAN1_Init() == 0)
  {
    Serial.println("Error initialising CAN!");
    while (1)
    {
    };
  }

  /* USER CODE BEGIN 2 */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Serial.println("Failed to config CAN Filter");
    while (1)
    {
    };
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* Start Error */
    Serial.println("Failed to start CAN");
    while (1)
    {
    };
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
    /* Notification Error */
    Serial.println("Failed to activate CAN notifications");
    while (1)
    {
    };
  }
}

void loop()
{
  // nothing happens after setup finishes.
}
