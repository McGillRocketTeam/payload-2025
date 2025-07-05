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
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "serial_monitor.h"
#include "ADC.h"
#include "accelerometer.h"
#include "CAN_bus.h"
#include "peltier.h"
#include "BME280.h"
#include "blink.h"
#include "SD_card.h"
#include "temperature.h"
#include "enabled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Timer purpose aliases
#define TIM_TEMPERATURE_SAMPLE htim2
#define TIM_PELTIER_REFERENCE htim3
#define TIM_PELTIER_PWM htim4
#define TIM_ADC_SAMPLE htim5
#define TIM_ACCELEROMETER_SAMPLE htim8
#define TIM_BLINK htim9
#define TIM_TELEMETRY htim12
// CAN conversion constants
#define TEMPERATURE_FACTOR 100 // Send temperature in centiCelsius to take advantage of the precision
#define BATTERY_VOLTAGE_SEND_MIN 10.0f // Volts
#define BATTERY_VOLTAGE_SEND_MAX 21.0f // Volts
#define BATTERY_VOLTAGE_SEND_FACTOR 256 // 2^8 is the maximum value for a uint8
#define VIBRATION_AMPLITUDE_FACTOR 10000 // Send amplitude in mV * 10 to take advantage of the precision

#define PELTIER_START_CYCLE 0.0f
// Error handling constants
#define MINOR_ERRORS_MAX 128
#define MINOR_ERROR_BLINK_TIME 100 // milliseconds
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BOOL_TO_ON(b) (b ? "on" : "off")
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
/* PL peripheral driver handlers ---------------------------------------------*/ 
PL_Accelerometer_Handler accelerometer;
PL_ADC_Handler adc;
PL_Blink_Handler blink;
PL_CANBus_Handler can;
PL_Peltier_Handler peltier;
PL_SDCard_Handler sd_card;

// Accelerometer buffer which DMA will write to. Needs to be shared a bit, so declared in main.
volatile uint16_t accelerometer_buffer[ACCELEROMETER_SAMPLE_SIZE_TRIPLE];

/* Global telemetry variables ------------------------------------------------*/
// BME280 variables
float temperature, pressure, humidity;
// Error Handling variables
bool ok;
uint8_t minor_errors;
uint32_t minor_error_last_time;
// ADC variables
float battery_voltage, cooler_current;
// FFT variables
float peak_amp_x, peak_amp_y, peak_amp_z;
float peak_freq_x, peak_freq_y, peak_freq_z;
// Temperature control
float target_temperature;
bool temperature_control_enabled;

// Flags for actions triggered by interrupts
volatile bool adc_new_sample_ready,
  BME280_sample_ready,
  blink_toggle_ready,
  minor_error_blink_toggle_ready,
  telemetry_report_ready;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C3_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */
/**
 * @brief Signals a critical error. Flags Payload as not ok, permanently turns on LD2. 
 * Calls `Error_Handler` if not in final build.
 * @note Critical errors occur if peripheral initialization fails, or in other similar instances.
 * @param category The category in which the critical error occurred.
 * @param msg A description of the critical error.
 */
void Critical_Error(enum log_category category, const char *msg);
/**
 * @brief Signals a minor error. Briefly blinks LD2. Keeps track of number of minor errors, 
 * and triggers `Critical_Error` if it exceeds `MINOR_ERRORS_MAX`.
 * @note Critical errors occur on single failures 
 * (i.e. one failed CAN message send or one failed peripheral sample).
 * @param category The category in which the minor error occurred.
 * @param msg A description of the minor error.
 */
void Minor_Error(enum log_category category, const char *msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C3_Init();
  MX_UART4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  PL_Log(LOG_GENERAL, LOG_NONE, LOG_INITIALIZING, "Initializing...");

  // Initialize error handling variables
  ok = 1;
  minor_errors = 0;
  minor_error_last_time = 0;
  minor_error_blink_toggle_ready = false;

  // Initialize temperature control variables
  target_temperature = TEMPERATURES[0]; // default to coldest temperature
  temperature_control_enabled = true; // default to cooling being on

  // Initialize and start CAN bus
  PL_Log(LOG_CAN_BUS, LOG_NONE, LOG_INITIALIZING, "Starting...");
  if (!PL_CANBus_Init(&can, &hcan1))
  {
    Critical_Error(LOG_CAN_BUS, "Initialization failed.");
  }
  PL_Log(LOG_CAN_BUS, LOG_NONE, LOG_OK, "Started.");

  // Initialize BME280 temperature sensor
  PL_Log(LOG_TEMPERATURE_SENSOR, LOG_NONE, LOG_INITIALIZING, "Starting...");
  PL_Log(LOG_TEMPERATURE_SENSOR, LOG_NONE, LOG_INITIALIZING, "Configuring BME280...");
  if (BME280_Config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16) != 0)
  {
    Critical_Error(LOG_TEMPERATURE_SENSOR, "BME280 configuration failed.");
  }
  PL_Log(LOG_TEMPERATURE_SENSOR, LOG_NONE, LOG_INITIALIZING, "Starting sample timer...");
  // Start temperature sample timer
  if (HAL_TIM_Base_Start_IT(&TIM_TEMPERATURE_SAMPLE) != HAL_OK)
  {
    Critical_Error(LOG_TEMPERATURE_SENSOR, "Sample timer start failed.");
  }
  // Initialize BME280 sample as ready to sample first time through main loop
  BME280_sample_ready = true;
  PL_Log(LOG_TEMPERATURE_SENSOR, LOG_NONE, LOG_OK, "Started.");

  // Initialize Peltier cooler PWM output
  PL_Log(LOG_PELTIER, LOG_NONE, LOG_INITIALIZING, "Initializing PWM output...");
  if (!PL_Peltier_Init(&peltier, &TIM_PELTIER_PWM, &TIM_PELTIER_REFERENCE, TIM_CHANNEL_1, TIM_CHANNEL_1))
  {
    Critical_Error(LOG_PELTIER, "PWM initialization failed");
  }
  // Start duty cycle at zero
  PL_Peltier_SetCycle(&peltier, PELTIER_START_CYCLE);
  PL_Log(LOG_PELTIER, LOG_NONE, LOG_OK, "Started with %d%% duty cycle.", (int)(PELTIER_START_CYCLE * 100));

  // Initialize ADCs
  PL_Log(LOG_ADC, LOG_NONE, LOG_INITIALIZING, "Starting...");
  if (!PL_ADC_Init(&adc, &hadc1, &hadc2, &hadc3, &TIM_ADC_SAMPLE, accelerometer_buffer))
  {
    Critical_Error(LOG_ADC, "Initialization failed.");
  }
  PL_Log(LOG_ADC, LOG_NONE, LOG_OK, "Started.");

  // Initialize and start accelerometer
  PL_Log(LOG_ACCELEROMETER, LOG_NONE, LOG_INITIALIZING, "Starting...");
  // Initialize accelerometer handler
  PL_Log(LOG_ACCELEROMETER, LOG_NONE, LOG_INITIALIZING, "Initializing handler...");
  if (!PL_Accelerometer_Init(&accelerometer, &TIM_ACCELEROMETER_SAMPLE, ACCEL_POWER_GPIO_Port, ACCEL_POWER_Pin))
  {
    Critical_Error(LOG_ACCELEROMETER, "Initialization failed.");
  }
  PL_Log(LOG_ACCELEROMETER, LOG_NONE, LOG_INITIALIZING, "Powering on accelerometer and starting sampling...");
  // Start the accelerometer
  if (!PL_Accelerometer_Start(&accelerometer))
  {
    Critical_Error(LOG_ACCELEROMETER, "Accelerometer start failed.");
  }
  PL_Log(LOG_ACCELEROMETER, LOG_NONE, LOG_OK, "Started.");

  // Initialize SD card
  PL_Log(LOG_SD_CARD, LOG_NONE, LOG_INITIALIZING, "Starting...");
  // Initialize SD card handler and mount file system
  PL_Log(LOG_SD_CARD, LOG_NONE, LOG_INITIALIZING, "Initializing and mounting...");
  if (!PL_SDCard_Init(&sd_card))
  {
    Critical_Error(LOG_SD_CARD, "Initialization/mount failed.");
  }
  // Open next log file on SD card
  PL_Log(LOG_SD_CARD, LOG_NONE, LOG_INITIALIZING, "Opening next log file...");
  if (!PL_SDCard_Open(&sd_card))
  {
    Critical_Error(LOG_SD_CARD, "Opening log file failed.");
  }
  PL_Log(LOG_SD_CARD, LOG_NONE, LOG_OK, "Started.");

  // Start the telemetry report timer
  PL_Log(LOG_GENERAL, LOG_NONE, LOG_INITIALIZING, "Starting telemetry report timer.");
  if (HAL_TIM_Base_Start_IT(&TIM_TELEMETRY) != HAL_OK)
  {
    Critical_Error(LOG_GENERAL, "Telemetry report timer start failed.");
  }
  // Initialize telemetry report as ready to send telemetry data first time through main loop
  telemetry_report_ready = true;
  PL_Log(LOG_GENERAL, LOG_NONE, LOG_OK, "Telemetry report timer started.");

  // Initialize blink driver handler
  PL_Log(LOG_BLINK, LOG_NONE, LOG_INITIALIZING, "Starting...");
  PL_Blink_Init(&blink, &TIM_BLINK, LD1_GPIO_Port, LD1_Pin);
  // Start blinking routine
  if (!PL_Blink_Start(&blink))
  {
    Critical_Error(LOG_BLINK, "Start error");
  }
  // Initialize blink as ready to turn LED on first time through main loop
  blink_toggle_ready = true;
  PL_Log(LOG_BLINK, LOG_NONE, LOG_OK, "Started.");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  PL_Log(LOG_GENERAL, LOG_NONE, LOG_OK, "Beginning main loop.");
  while (1)
  {
    /* Gather data before communication functions --------------------------------*/
    if (adc_new_sample_ready)
    {
      // Calculate values from ADC reading
      battery_voltage = PL_ADC_GetBatteryVoltage(&adc);
      cooler_current = PL_ADC_GetCoolerCurrent(&adc);

      PL_Log(LOG_ADC, LOG_NONE, LOG_OK, "Battery Voltage: %7d mV | Cooler Current: %5d mA",
             (int)(1000 * battery_voltage),
             (int)(1000 * cooler_current));

      adc_new_sample_ready = false;
    }

    if (BME280_sample_ready)
    {
      BME280_Measure();
      PL_Log(LOG_TEMPERATURE_SENSOR, LOG_NONE, LOG_OK,
             "Temperature: %3d C | Pressure: %7d Pa | Humidity: %3d%%",
             (int)temperature,
             (int)pressure,
             (int)humidity);
      BME280_sample_ready = false;
    }

    /* Communicate with the outside world and other devices ----------------------*/
    if (accelerometer.analysis_ready)
    {
      // Write new accelerometer data to the SD card
      if (!PL_SDCard_WriteAccelerometer(&sd_card,
                                        HAL_GetTick(),
                                        (uint16_t *)accelerometer.fft_buffer_x,
                                        (uint16_t *)accelerometer.fft_buffer_y,
                                        (uint16_t *)accelerometer.fft_buffer_z))
      {
        Minor_Error(LOG_SD_CARD, "Accelerometer packet write failed.");
      }
      else
      {
        PL_Log(LOG_SD_CARD, LOG_ACCELEROMETER, LOG_OK, "Wrote accelerometer packet.");
      }

      // Perform FFT analysis (stored in amplitude buffers)
      PL_Accelerometer_Analyze(&accelerometer);
      // Find peak amplitudes and frequencies on each axis
      peak_freq_x = PL_Accelerometer_PeakFrequency(accelerometer.amplitudes_x, &peak_amp_x);
      peak_freq_y = PL_Accelerometer_PeakFrequency(accelerometer.amplitudes_y, &peak_amp_y);
      peak_freq_z = PL_Accelerometer_PeakFrequency(accelerometer.amplitudes_z, &peak_amp_z);

      PL_Log(LOG_ACCELEROMETER, LOG_NONE, LOG_OK,
             "Peak Frequency X: %5d Hz | Peak Amplitude X: %5d mV",
             (int)peak_freq_x,
             (int)(1000 * peak_amp_x));
      PL_Log(LOG_ACCELEROMETER, LOG_NONE, LOG_OK,
             "Peak Frequency Y: %5d Hz | Peak Amplitude Y: %5d mV",
             (int)peak_freq_y,
             (int)(1000 * peak_amp_y));
      PL_Log(LOG_ACCELEROMETER, LOG_NONE, LOG_OK,
             "Peak Frequency Z: %5d Hz | Peak Amplitude Z: %5d mV",
             (int)peak_freq_z,
             (int)(1000 * peak_amp_z));
    }

    if (can.command_ready)
    {
      struct command com = PL_CANBus_ParseCommand(&can);
      switch (com.type)
      {
      case RESET_PAYLOAD:
        PL_Log(LOG_CAN_BUS, LOG_GENERAL, LOG_OK, "Payload reset.");
        // Reset STM32 via toggling buck reset
        HAL_GPIO_TogglePin(BUCK_GPIO_Port, BUCK_Pin);
        // Wait a short delay to ensure signal stabilizes
        HAL_Delay(10);
        // Toggle again to provide both rising and falling edges (we're not sure which one is necessary)
        HAL_GPIO_TogglePin(BUCK_GPIO_Port, BUCK_Pin);
        break;
      case TOGGLE_SAMPLING:
        // Call proper accelerometer function depending on toggle parameter, then check return type
        if (!(com.data.on
                  ? PL_Accelerometer_Start(&accelerometer)
                  : PL_Accelerometer_Stop(&accelerometer)))
        {
          // Format error message since `Minor_Error` isn't variadic
          char msg[64];
          snprintf(msg, sizeof(msg), "Switching accelerometer to %s failed.", BOOL_TO_ON(com.data.on));
          Minor_Error(LOG_CAN_BUS, msg);
        }
        else
        {
          PL_Log(LOG_CAN_BUS, LOG_ACCELEROMETER, LOG_OK,
                 "Accelerometer switched to %s.", BOOL_TO_ON(com.data.on));
        }
        break;
      case TOGGLE_COOLER:
        temperature_control_enabled = com.data.on;
        PL_Log(LOG_CAN_BUS, LOG_TEMPERATURE_SENSOR, LOG_OK,
               "Temperature control switched to %s.", BOOL_TO_ON(com.data.on));
        break;
      case TOGGLE_LAUNCH_MODE:
        // Launch mode doesn't do anything since we don't need it anymore
        PL_Log(LOG_CAN_BUS, LOG_GENERAL, LOG_OK, "Launch mode switched to %s.", BOOL_TO_ON(com.data.on));
        break;
      case LANDED:
        PL_Log(LOG_CAN_BUS, LOG_GENERAL, LOG_OK, "Landed.");
        break;
      case SET_TEMPERATURE:
        target_temperature = com.data.temp;
        PL_Log(LOG_CAN_BUS, LOG_TEMPERATURE_SENSOR | LOG_PELTIER, LOG_OK,
               "Target temperature set to %d.", (int)target_temperature);
        break;
      case NONE:
        PL_Log(LOG_CAN_BUS, LOG_NONE, LOG_WARNING, "No command received.");
        break;
      case INVALID:
        Minor_Error(LOG_CAN_BUS, "Invalid command received.");
        break;
      }
    }

    if (telemetry_report_ready)
    {
      if (!PL_SDCard_WriteTelemetry(&sd_card,
                                    HAL_GetTick(),
                                    ok,
                                    accelerometer.sampling,
                                    temperature_control_enabled,
                                    target_temperature,
                                    temperature,
                                    pressure,
                                    humidity,
                                    battery_voltage))
      {
        Minor_Error(LOG_SD_CARD, "Telemetry packet write error.");
      }
      else
      {
        PL_Log(LOG_SD_CARD, LOG_NONE, LOG_OK, "Wrote telemetry packet.");
      }

      // Determine target temperature
      uint8_t target_temperature_index = 0;
      for (int i = 0; i < N_TEMPERATURES; i++)
      {
        if (target_temperature == TEMPERATURES[i])
        {
          target_temperature_index = i;
        }
      }
      // Send CAN message
      if (!PL_CANBus_Send(&can,
                          ok,
                          accelerometer.sampling,
                          temperature_control_enabled,
                          target_temperature_index,
                          // Multiply temperature to take advantage of the precision
                          (int16_t)roundf(temperature * TEMPERATURE_FACTOR),
                          // Compress battery voltage into uint8 to take advantage of the precision
                          (uint8_t)roundf(
                              BATTERY_VOLTAGE_SEND_FACTOR *
                              (battery_voltage - BATTERY_VOLTAGE_SEND_MIN) /
                              (BATTERY_VOLTAGE_SEND_MAX - BATTERY_VOLTAGE_SEND_MIN)),
                          // Round to nearest Hz
                          (uint16_t)roundf(peak_freq_x),
                          (uint16_t)roundf(peak_freq_y),
                          (uint16_t)roundf(peak_freq_z),
                          // Multiply amplitudes to take advantage of the precision
                          (uint16_t)roundf(peak_amp_x * VIBRATION_AMPLITUDE_FACTOR),
                          (uint16_t)roundf(peak_amp_y * VIBRATION_AMPLITUDE_FACTOR),
                          (uint16_t)roundf(peak_amp_z * VIBRATION_AMPLITUDE_FACTOR),
                          HAL_GetTick()))
      {
        Minor_Error(LOG_CAN_BUS, "Failed to send set of three messages.");
      }
      else
      {
        PL_Log(LOG_CAN_BUS, LOG_NONE, LOG_OK, "Set of three messages sent.");
      }

      telemetry_report_ready = false;
    }

    if (blink_toggle_ready)
    {
      // Only log if the light was actually blinked
      if (PL_Blink_Toggle(&blink))
      {
        PL_Log(LOG_BLINK, LOG_NONE, LOG_OK, "Blinked.");
      }
      blink_toggle_ready = false;
    }

    if (minor_error_blink_toggle_ready && HAL_GetTick() - minor_error_last_time >= MINOR_ERROR_BLINK_TIME)
    {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      minor_error_blink_toggle_ready = false; // Reset blink ready flag
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

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
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_TRIPLEMODE_REGSIMULT_INJECSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_1;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_56CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISINGFALLING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T5_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
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
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_56CYCLES;
  sConfigInjected.AutoInjectedConv = ENABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
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
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
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
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
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

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1440;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 4;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1440;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 7200-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 5000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 72-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 100-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 7200-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 7200-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 4000-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ACCEL_POWER_Pin|LD1_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUCK_GPIO_Port, BUCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ACCEL_POWER_Pin LD1_Pin LD2_Pin */
  GPIO_InitStruct.Pin = ACCEL_POWER_Pin|LD1_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : COOLER_SHORT_Pin */
  GPIO_InitStruct.Pin = COOLER_SHORT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(COOLER_SHORT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_DETECT_Pin BUTTON_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin|BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUCK_Pin */
  GPIO_InitStruct.Pin = BUCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUCK_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
  if (!PL_CANBus_Receive(&can))
  {
    Minor_Error(LOG_CAN_BUS, "Failed to receive message.");
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM_ADC_SAMPLE.Instance)
  {
    PL_ADC_InjectedConversion(&adc);
    adc_new_sample_ready = true;
  }
  else if (htim->Instance == TIM_BLINK.Instance)
  {
    blink_toggle_ready = true;
  }
  else if (htim->Instance == TIM_TEMPERATURE_SAMPLE.Instance)
  {
    BME280_sample_ready = true;
  }
  else if (htim->Instance == TIM_TELEMETRY.Instance)
  {
    telemetry_report_ready = true;
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    PL_Accelerometer_Record(&accelerometer, accelerometer_buffer);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    PL_Accelerometer_Record(&accelerometer, &accelerometer_buffer[FFT_SIZE_TRIPLE]);
  }
}

void Critical_Error(enum log_category category, const char *msg)
{
  ok = 0;
  // Turn on LD2 to indicate critical error
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  PL_Log(category, LOG_NONE, LOG_ERROR, msg);
#if !FINAL_BUILD
  Error_Handler();
#endif
}

void Minor_Error(enum log_category category, const char *msg)
{
  if (ok)
  {
    minor_errors++;
    /* 
     * Color number of minor errors depending on how many occurred.
     * < 1/3 of max -> COLOR_OK
     * < 2/3 of max -> COLOR_WARNING
     * < 3/3 of max -> COLOR_ERROR
     */
    char *err_num_color;
    if (minor_errors < MINOR_ERRORS_MAX / 3)
    {
      err_num_color = COLOR_OK;
    }
    else if (minor_errors >= MINOR_ERRORS_MAX / 3 && minor_errors < 2 * MINOR_ERRORS_MAX / 3)
    {
      err_num_color = COLOR_WARNING;
    }
    else
    {
      err_num_color = COLOR_ERROR;
    }
    PL_Log(category, LOG_NONE, LOG_WARNING,
           "%s[%sME:%s%3d%s%s]%s %s",
           COLOR_WHITE,
           COLOR_RESET,
           err_num_color,
           minor_errors,
           COLOR_RESET,
           COLOR_WHITE,
           COLOR_RESET,
           msg);
    if (minor_errors >= MINOR_ERRORS_MAX)
    {
      Critical_Error(LOG_GENERAL, "Minor error excess.");
    }
    else
    {
      minor_error_last_time = HAL_GetTick();
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      minor_error_blink_toggle_ready = true; // Set flag to toggle LD2 off after `MINOR_ERROR_BLINK_TIME`
    }
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
