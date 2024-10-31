/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "sdcard.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h> //for va_list var arg functions
#include <stdbool.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FREQ_X_LENGTH 14
#define FREQ_Y_LENGTH 14
#define FREQ_Z_LENGTH 14
#define A_X_LENGTH 9
#define A_Y_LENGTH 9
#define A_Z_LENGTH 9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM13_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// flags
volatile uint8_t rtf = 0;       // Ready To Flush
volatile uint8_t halfcpltx = 0; // ADC half complete
volatile uint8_t halfcplty = 0;
volatile uint8_t halfcpltz = 0;
volatile uint8_t cpltx = 0; // ADC complete
volatile uint8_t cplty = 0;
volatile uint8_t cpltz = 0;
volatile int receive_flag = 0;

// ADC buffer
uint16_t adc_buffer_x[16+ADC_BUFFER_SIZE] = {0};
uint16_t adc_buffer_y[16+ADC_BUFFER_SIZE] = {0};
uint16_t adc_buffer_z[16+ADC_BUFFER_SIZE] = {0};

// CAN variables
uint8_t init_buffer[27] = "FC B activated!\n\r";
uint8_t tx_buffer[27] = "Transmit Board 3!\n\r";
uint8_t rx_buffer[27] = "Received Board 3!\n\r";

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

CAN_TxHeaderTypeDef TxHeadermsg1;
CAN_TxHeaderTypeDef TxHeadermsg2;

uint32_t TxMailbox[4];

uint8_t TxData[8];
uint8_t RxData[8];

uint8_t TxDatamsg1[8];
uint8_t TxDatamsg2[8];

uint8_t count = 0;
uint8_t toggle = 0;

uint64_t currenttime = 0;
uint64_t lasttime = 0;

bool isSampling = true;
union my_msg
{
  uint64_t msg;
  uint8_t bytes[8];
};

struct Data
{
  float frqX;
  float frqY;
  float frqZ;

  float ampX;
  float ampY;
  float ampZ;

  bool isSampling;

  uint32_t seconds;
};

uint64_t formatMsg1(bool isSampling, float frqX, float frqY, float frqZ, float ampX, float ampY)
{
  uint64_t formattedMsg1;

  uint64_t samplingBit; //"converting" a boolean to a 0 or 1
  if (isSampling == true)
  {
    samplingBit = 1;
  }
  else
  {
    samplingBit = 0;
  }
  // round the frequencies to the nearest unit before shifting
  uint64_t roundedFrqZ = roundf(frqZ);
  if (frqZ > 16383)
    roundedFrqZ = 16383; // use 11111111111111 to indicate overflow
  uint64_t roundedFrqY = roundf(frqY);
  if (frqY > 16383)
    roundedFrqY = 16383; // use 11111111111111 to indicate overflow
  uint64_t roundedFrqX = roundf(frqX);
  if (frqX > 16383)
    roundedFrqX = 16383; // use 11111111111111 to indicate overflow
  // Value is x100 and %100 at groundstation to get 2 decimal places. Max value is 37m/s^2 with gain 4.34x, so we x13 to use full 511 range
  uint64_t roundedAmpX = roundf(ampX * 100 * 13);
  if (roundedAmpX > 511)
    roundedAmpX = 511; // use 11111111 to indicate overflow
  // Value is x100 and %100 at groundstation to get 2 decimal places. Max value is 80.5m/s^2 with gain 2x, so we x6 to use full 511 range
  uint64_t roundedAmpY = roundf(ampY * 100 * 6);
  if (roundedAmpY > 511)
    roundedAmpY = 511; // use 11111111 to indicate overflow
  formattedMsg1 =
      (samplingBit) | ((roundedFrqX & 16383) << 1) | ((roundedFrqY & 16383) << 15) | ((roundedFrqZ & 16383) << 29) | ((roundedAmpX & 511) << 46) | ((roundedAmpY & 511) << 55);
  return formattedMsg1;
}

uint64_t formatMsg2(float ampZ, uint32_t seconds)
{
  uint64_t formattedMsg2;
  // Value is x100 and %100 at groundstation to get 2 decimal places. Max value is 161m/s^2 with gain 1x, so we x3 to use full 511 range
  uint64_t roundedAmpZ = roundf(ampZ * 100 * 3); // Data on grounstation is scaled by 3,6,13 times compared to SD card
  if (roundedAmpZ > 511)
    roundedAmpZ = 511; // use 11111111 to indicate overflow
  uint64_t formattedSeconds = 0;
  if (seconds > 32768)
  { // Overflow seconds back to zero if it reaches more than 15 bits.
    formattedSeconds = seconds - 32768;
  }
  if (seconds > 65535)
  { // if we are still over 15 bits then fix the value (this happens after 65,536 seconds
    formattedSeconds = 32767;
  }
  else
  {
    formattedSeconds = (uint64_t)seconds;
  }
  formattedMsg2 = (formattedSeconds & 32767) | ((roundedAmpZ & 511) << 15) | (0 << 24);
  return formattedMsg2;
}

void buildMsg(union my_msg *uMsg1, union my_msg *uMsg2, struct Data dt)
{

  uint64_t msg1 = formatMsg1(dt.isSampling, dt.frqX, dt.frqY, dt.frqZ, dt.ampX, dt.ampY);
  uint64_t msg2 = formatMsg2(dt.ampZ, dt.seconds);
  uMsg1->msg = msg1;
  uMsg2->msg = msg2;
}

// Function to create a new file with an incremented number
void create_new_filename(char *filename, const char *basename, const char *extension)
{
  UINT num = 1;
  FILINFO fno;
  while (1)
  {
    snprintf(filename, 32, "%s%d.%s", basename, num, extension);
    if (f_stat(filename, &fno) != FR_OK)
    {
      // File doesn't exist
      break;
    }
    num++;
  }
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_FATFS_Init();
  MX_TIM14_Init();
  MX_TIM13_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  myprintf("\r\n~ Starting... ~\r\n\r\n");

  HAL_GPIO_WritePin(LED_INIT_GPIO_Port, LED_INIT_Pin, GPIO_PIN_RESET); // Init LED Off

  HAL_Delay(1000); // a short delay is important to let the SD card settle

  HAL_StatusTypeDef status = HAL_OK;

  /* Start the CAN peripheral */
  /*
  CAN bus abandoned for now -> not starting them

  status = HAL_CAN_Start(&hcan1);
  if (status != HAL_OK)
  {
    myprintf("HAL_CAN_Start error (%i)\r\n", status);
    while (1);
  }

  status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  if (status != HAL_OK)
  {
    myprintf("HAL_CAN_ActivateNotification error (%i)\r\n", status);
    while (1);
  }

  */

  TxHeader.DLC = 2;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x333;
  TxHeader.TransmitGlobalTime = DISABLE;

  TxHeadermsg1.DLC = 8;
  TxHeadermsg1.ExtId = 0;
  TxHeadermsg1.IDE = CAN_ID_STD;
  TxHeadermsg1.RTR = CAN_RTR_DATA;
  TxHeadermsg1.StdId = 0x300;
  TxHeadermsg1.TransmitGlobalTime = DISABLE;

  TxHeadermsg2.DLC = 8;
  TxHeadermsg2.ExtId = 0;
  TxHeadermsg2.IDE = CAN_ID_STD;
  TxHeadermsg2.RTR = CAN_RTR_DATA;
  TxHeadermsg2.StdId = 0x301;
  TxHeadermsg2.TransmitGlobalTime = DISABLE;

  /* Initialize the SD card */

  // some variables for FatFs
  FATFS FatFs;  // Fatfs handle
  FRESULT fres; // Result after operations

  // Open the file system
  myprintf("\r\n~ Mounting... ~\r\n\r\n");
  fres = f_mount(&FatFs, "", 1);
  if (fres != FR_OK)
  {
    myprintf("f_mount error (%i)\r\n", fres);
    while (1);
  }

  // Create new file name
  char filename[12];
  create_new_filename(filename, "DATA", "txt");

  //Open CSV
  fres = f_open(&fil, filename, FA_CREATE_NEW | FA_WRITE);
  if (fres == FR_OK)
  {
    myprintf("I was able to open '%s' for writing\r\n", filename);
  }
  else
  {
    myprintf("f_open error (%i)\r\n", fres);
    while (1);
  }

  status = HAL_TIM_Base_Start_IT(&htim14);
  if (status != HAL_OK)
  {
    myprintf("HAL_TIM_Base_Start_IT error (%i)\r\n", status);
    while (1);
  }

  /* Start the ADC */
  // Write metadata headers as
  // `"amplitudeX.time:"  [time in ms (4 bytes)]  "block:"  [block size in bytes (4 bytes)]  ".\0"`
  sprintf(adc_buffer_x, "amplitudeX.time:");
  sprintf(adc_buffer_x + 10, "block:");
  adc_buffer_x[13] = (uint16_t)(ADC_BUFFER_SIZE & 0xFFFF);
  adc_buffer_x[14] = (uint16_t)((ADC_BUFFER_SIZE >> 16) & 0xFFFF);
  sprintf(adc_buffer_x + 15, ".");
  sprintf(adc_buffer_y, "amplitudeY.time:");
  sprintf(adc_buffer_y + 10, "block:");
  adc_buffer_y[13] = (uint16_t)(ADC_BUFFER_SIZE & 0xFFFF);
  adc_buffer_y[14] = (uint16_t)((ADC_BUFFER_SIZE >> 16) & 0xFFFF);
  sprintf(adc_buffer_y + 15, ".");
  sprintf(adc_buffer_z, "amplitudeZ.time:");
  sprintf(adc_buffer_z + 10, "block:");
  adc_buffer_z[13] = (uint16_t)(ADC_BUFFER_SIZE & 0xFFFF);
  adc_buffer_z[14] = (uint16_t)((ADC_BUFFER_SIZE >> 16) & 0xFFFF);
  sprintf(adc_buffer_z + 15, ".");

  status = HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &adc_buffer_x[16], ADC_BUFFER_SIZE);
  if (status != HAL_OK)
  {
    myprintf("HAL_ADC_Start_DMA adc1 error (%i)\r\n", status);
    while (1);
  }
  status = HAL_ADC_Start_DMA(&hadc2, (uint32_t *) &adc_buffer_y[16], ADC_BUFFER_SIZE);
  if (status != HAL_OK)
  {
    myprintf("HAL_ADC_Start_DMA adc2 error (%i)\r\n", status);
    while (1);
  }
  status = HAL_ADC_Start_DMA(&hadc3, (uint32_t *) &adc_buffer_z[16], ADC_BUFFER_SIZE);
  if (status != HAL_OK)
  {
    myprintf("HAL_ADC_Start_DMA adc3 error (%i)\r\n", status);
    while (1);
  }

  /* Start the sampling timer */
  status = HAL_TIM_Base_Start_IT(&htim2);
  // status = HAL_TIM_Base_Start_IT(&htim13);
  if (status != HAL_OK)
  {
    myprintf("HAL_TIM_Base_Start_IT error (%i)\r\n", status);
    while (1);
  }

  HAL_GPIO_WritePin(LED_INIT_GPIO_Port, LED_INIT_Pin, GPIO_PIN_SET); // Init LED On

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (receive_flag == 1)
    {
      myprintf("Received %d\n\r", RxData[1]);
      uint32_t id = RxHeader.StdId;
      myprintf("Received message with Standard ID: 0x%03X\n\r", id);

      if (id == 0x011 || id == 0x012)
      {
        if (RxData[0] == 1)
        {
          myprintf("Payload is now sampling\n\r");
          isSampling = true;
        }
        if (RxData[0] == 0)
        {
          myprintf("Payload stopped sampling\n\r");
          isSampling = false;
        }
      }
      if (id == 0x005 || id == 0x006)
      {
        myprintf("Payload is now Scrub aka Reset\n\r");
      }

      receive_flag = 0;
    }
    if (HAL_GetTick() - lasttime > 1200)
    { // 1.2Hz
      lasttime = HAL_GetTick();
      // msg1
      // bool isSampling = true;
      float frqX = 1112.82f;
      float frqY = 2404.91f;
      float frqZ = 13711.42f;

      float ampX = 0.34f;
      float ampY = 0.412f;
      float ampZ = 1.664f;

      if (isSampling == false)
      {
        frqX = 0;
        frqY = 0;
        frqZ = 0;

        ampX = 0;
        ampY = 0;
        ampZ = 0;
      }

      uint32_t seconds = HAL_GetTick();
      seconds = seconds / 1000;

      struct Data data_packet = {frqX, frqY, frqZ, ampX, ampY, ampZ, isSampling, seconds};

      union my_msg m1;
      union my_msg m2;

      buildMsg(&m1, &m2, data_packet);
      HAL_CAN_AddTxMessage(&hcan1, &TxHeadermsg1, m1.bytes, &TxMailbox[0]);
      // HAL_Delay(100);
      HAL_CAN_AddTxMessage(&hcan1, &TxHeadermsg2, m2.bytes, &TxMailbox[0]);
    }

    if (rtf == 1)
    {
      SDCard_file_flush_line_check(&fil, &line_written);
      rtf = 0;
    }

    if (halfcpltx == 1)
    {
      SDCard_file_write(&fil, adc_buffer_x, (uint32_t) 32+ADC_BUFFER_SIZE, &line_written);

      myprintf("Half complete X timestamp: %lu\n\r", adc_buffer_x[8] + (adc_buffer_x[9] << 16));

      halfcpltx = 0;
    }
    if (halfcplty == 1)
    {
      SDCard_file_write(&fil, adc_buffer_y, (uint32_t) 32+ADC_BUFFER_SIZE, &line_written);

      myprintf("Half complete Y timestamp: %lu\n\r", adc_buffer_y[8] + (adc_buffer_y[9] << 16));

      halfcplty = 0;
    }
    if (halfcpltz == 1)
    {
      SDCard_file_write(&fil, adc_buffer_z, (uint32_t) 32+ADC_BUFFER_SIZE, &line_written);

      myprintf("Half complete Z timestamp: %lu\n\r", adc_buffer_z[8] + (adc_buffer_z[9] << 16));

      halfcpltz = 0;
    }

    if (cpltx == 1)
    {
      uint16_t adc_buffer_last_half[16+(ADC_BUFFER_SIZE/2)];
      memcpy(adc_buffer_last_half, adc_buffer_x, 16 * sizeof(uint16_t));
      memcpy(adc_buffer_last_half + 16, adc_buffer_x + 16 + (ADC_BUFFER_SIZE/2), (ADC_BUFFER_SIZE/2) * sizeof(uint16_t));

      SDCard_file_write(&fil, adc_buffer_last_half, (uint32_t) 32+ADC_BUFFER_SIZE, &line_written);

      myprintf("Complete X timestamp: %lu\n\r", adc_buffer_last_half[8] + (adc_buffer_last_half[9] << 16));

      cpltx = 0;
    }
    if (cplty == 1)
    {
      uint16_t adc_buffer_last_half[16+(ADC_BUFFER_SIZE/2)];
      memcpy(adc_buffer_last_half, adc_buffer_y, 16 * sizeof(uint16_t));
      memcpy(adc_buffer_last_half + 16, adc_buffer_y + 16 + (ADC_BUFFER_SIZE/2), (ADC_BUFFER_SIZE/2) * sizeof(uint16_t));

      SDCard_file_write(&fil, adc_buffer_last_half, (uint32_t) 32+ADC_BUFFER_SIZE, &line_written);

      myprintf("Complete Y timestamp: %lu\n\r", adc_buffer_last_half[8] + (adc_buffer_last_half[9] << 16));

      cplty = 0;
    }
    if (cpltz == 1)
    {
      uint16_t adc_buffer_last_half[16+(ADC_BUFFER_SIZE/2)];
      memcpy(adc_buffer_last_half, adc_buffer_z, 16 * sizeof(uint16_t));
      memcpy(adc_buffer_last_half + 16, adc_buffer_z + 16 + (ADC_BUFFER_SIZE/2), (ADC_BUFFER_SIZE/2) * sizeof(uint16_t));

      SDCard_file_write(&fil, adc_buffer_last_half, (uint32_t) 32+ADC_BUFFER_SIZE, &line_written);

      myprintf("Complete Z timestamp: %lu\n\r", adc_buffer_last_half[8] + (adc_buffer_last_half[9] << 16));

      cpltz = 0;
    }

    if (HAL_GetTick() > TERMINATION_TIME_MS)
    {
      f_close(&fil);
      myprintf("\r\n~ Closing file... ~\r\n\r\n");
      f_mount(NULL, "", 0);
      myprintf("\r\n~ Unmounting... ~\r\n\r\n");
      HAL_TIM_Base_Stop_IT(&htim14);
      myprintf("\r\n~ Stopping... ~\r\n\r\n");
      HAL_ADC_Stop_DMA(&hadc1);
      HAL_ADC_Stop_DMA(&hadc2);
      HAL_ADC_Stop_DMA(&hadc3);
      HAL_TIM_Base_Stop_IT(&htim2);
      HAL_GPIO_WritePin(LED_INIT_GPIO_Port, LED_INIT_Pin, GPIO_PIN_RESET); // Init LED Off
      while (1);
    }

  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 20;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;  // anything between 0 to SlaveStartFilterBank
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x011<<5;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x011<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 1;  // anything between 0 to SlaveStartFilterBank
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x012<<5;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x012<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 2;  // anything between 0 to SlaveStartFilterBank
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x005<<5;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x005<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 3;  // anything between 0 to SlaveStartFilterBank
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0x006<<5;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x006<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1
  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 84-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 100-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 8400-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_INIT_GPIO_Port, LED_INIT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_INIT_Pin */
  GPIO_InitStruct.Pin = LED_INIT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_INIT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim14)
  {
    if (flush_timer == SD_FLUSH_TIME_FREQ_SEC)
    {
      rtf = 1;
      flush_timer = 0;
    }
    else
      flush_timer++;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
  HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
  count++;
  // myprintf("Received %d\n\r",RxData[1]);
  receive_flag = 1;
}

// Called when first half of buffer is filled
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    const uint32_t timestamp = HAL_GetTick();
    adc_buffer_x[8] = (uint16_t)(timestamp & 0xFFFF);         // Lower 16 bits
    adc_buffer_x[9] = (uint16_t)((timestamp >> 16) & 0xFFFF); // Upper 16 bits

    halfcpltx = 1;
  }
  else if (hadc == &hadc2)
  {
    const uint32_t timestamp = HAL_GetTick();
    adc_buffer_y[8] = (uint16_t)(timestamp & 0xFFFF);         // Lower 16 bits
    adc_buffer_y[9] = (uint16_t)((timestamp >> 16) & 0xFFFF); // Upper 16 bits

    halfcplty = 1;
  }
  else if (hadc == &hadc3)
  {
    const uint32_t timestamp = HAL_GetTick();
    adc_buffer_z[8] = (uint16_t)(timestamp & 0xFFFF);         // Lower 16 bits
    adc_buffer_z[9] = (uint16_t)((timestamp >> 16) & 0xFFFF); // Upper 16 bits

    halfcpltz = 1;
  }
}

// Called when buffer is completely filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    const uint32_t timestamp = HAL_GetTick();
    adc_buffer_x[8] = (uint16_t)(timestamp & 0xFFFF);         // Lower 16 bits
    adc_buffer_x[9] = (uint16_t)((timestamp >> 16) & 0xFFFF); // Upper 16 bits

    cpltx = 1;
  }
  else if (hadc == &hadc2)
  {
    const uint32_t timestamp = HAL_GetTick();
    adc_buffer_y[8] = (uint16_t)(timestamp & 0xFFFF);         // Lower 16 bits
    adc_buffer_y[9] = (uint16_t)((timestamp >> 16) & 0xFFFF); // Upper 16 bits

    cplty = 1;
  }
  else if (hadc == &hadc3)
  {
    const uint32_t timestamp = HAL_GetTick();
    adc_buffer_z[8] = (uint16_t)(timestamp & 0xFFFF);         // Lower 16 bits
    adc_buffer_z[9] = (uint16_t)((timestamp >> 16) & 0xFFFF); // Upper 16 bits

    cpltz = 1;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
