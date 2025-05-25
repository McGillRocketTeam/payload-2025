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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "serial_monitor.h"
#include "CAN_bus.h"
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
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct CAN_bus_handler can;

uint32_t TxMailbox;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	CAN_bus_receive(&can);
	struct command com = CAN_bus_parse_command(&can);
	char *type;
	int data = 0;
	switch (com.type)
	{
	case RESET_PAYLOAD:
		type = "RESET_PAYLOAD";
		break;
	case TOGGLE_SAMPLING:
		type = "TOGGLE_SAMPLING";
		data = com.data.on;
		break;
	case TOGGLE_COOLER:
		type = "TOGGLE_COOLER";
		data = com.data.on;
		break;
	case LANDED:
		type = "LANDED";
		break;
	case SET_TEMPERATURE:
		type = "SET_TEMPERATURE";
		data = com.data.temp;
		break;
	case NONE:
		type = "NONE";
		break;
	case INVALID:
		type = "INVALID";
		break;
	}
	printf("| PL -------------------------------\r\n");
	printf("| Command received: %s, Data: %d\r\n", type, data);
	printf("| ----------------------------------\r\n\r\n");
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
	printf("| AV -------------------------------\r\n");
	while (HAL_CAN_GetRxFifoFillLevel(hcan1, CAN_RX_FIFO1) > 0)
	{
		HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO1, &RxHeader, RxData);
		switch (RxHeader.StdId)
		{
		case 0x200:
			union CAN_msg_1_u msg1;
			memcpy(&msg1, RxData, sizeof(RxData));
			printf("| Received message 1. ok: %d, sampling state: %d, temperature control state: %d, target temp: %d, current temp: %d, battery voltage: %d\r\n",
					msg1.msg.ok,
					msg1.msg.sampling_state,
					msg1.msg.temp_ctrl_state,
					msg1.msg.target_temp,
					msg1.msg.current_temp,
					msg1.msg.battery_voltage);
			break;
		case 0x201:
			union CAN_msg_2_u msg2;
			memcpy(&msg2, RxData, sizeof(RxData));
			printf("| Received message 2. x frequency: %d, y frequency: %d, z frequency: %d, x amplitude: %d\r\n",
					msg2.msg.frqX,
					msg2.msg.frqY,
					msg2.msg.frqZ,
					msg2.msg.ampX);
			break;
		case 0x202:
			union CAN_msg_3_u msg3;
			memcpy(&msg3, RxData, sizeof(RxData));
			printf("| Received message 3. y amplitude: %d, z amplitude: %d, time: %ld\r\n",
					msg3.msg.ampY,
					msg3.msg.ampZ,
					msg3.msg.time_elapsed);
			break;
		}
	}
	printf("| ----------------------------------\r\n\r\n");
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
  MX_CAN1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n\r\n\r\nInitializing...\r\n");

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
	  printf("activate notification error\r\n");
	  Error_Handler();
  }
  if (!CAN_bus_init(&can, &hcan1, 0x200))
  {
	  printf("can bus init error\r\n");
	  Error_Handler();
  }

  int count = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (!CAN_bus_send(&can, 1, 0, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9, HAL_GetTick()))
	{
		printf("can bus send error\r\n");
		Error_Handler();
	}

	CAN_TxHeaderTypeDef TxHeader;

	TxHeader.DLC = 1;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.TransmitGlobalTime = DISABLE;

	uint8_t TxData[8];

	switch (count)
	{
	case 0:
		TxHeader.StdId = RESET_PAYLOAD;
		break;
	case 1:
		TxHeader.StdId = TOGGLE_SAMPLING;
		TxData[0] = 1;
		break;
	case 2:
		TxHeader.StdId = TOGGLE_SAMPLING;
		TxData[0] = 0;
		break;
	case 3:
		TxHeader.StdId = TOGGLE_COOLER;
		TxData[0] = 1;
		break;
	case 4:
		TxHeader.StdId = TOGGLE_COOLER;
		TxData[0] = 0;
		break;
	case 5:
		TxHeader.StdId = LANDED;
		break;
	case 6:
		TxHeader.StdId = SET_TEMPERATURE;
		TxData[0] = 0;
		break;
	case 7:
		TxHeader.StdId = SET_TEMPERATURE;
		TxData[0] = 1;
		break;
	case 8:
		TxHeader.StdId = SET_TEMPERATURE;
		TxData[0] = 2;
		break;
	case 9:
		TxHeader.StdId = SET_TEMPERATURE;
		TxData[0] = 3;
		break;
	case 10:
		TxHeader.StdId = SET_TEMPERATURE;
		TxData[0] = 4;
		break;
	case 11:
		TxHeader.StdId = SET_TEMPERATURE;
		TxData[0] = 5;
		break;
	case 12:
		TxHeader.StdId = SET_TEMPERATURE;
		TxData[0] = 6;
		break;
	case 13:
		TxHeader.StdId = SET_TEMPERATURE;
		TxData[0] = 7;
		break;
	}

	HAL_Delay(1000);

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		printf("can bus add tx message error\r\n");
		Error_Handler();
	}

	count++;
	count %= 14;

	HAL_Delay(1000);
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
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
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
    canfilterconfig.FilterBank = 10;  // anything between 0 to SlaveStartFilterBank
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO1;
    canfilterconfig.FilterIdHigh = 0x200 << 5;
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0xFFFF << 5;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1

    HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 11;  // anything between 0 to SlaveStartFilterBank
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO1;
    canfilterconfig.FilterIdHigh = 0x201 << 5;
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0xFFFF << 5;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1

    HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 12;  // anything between 0 to SlaveStartFilterBank
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO1;
    canfilterconfig.FilterIdHigh = 0x202 << 5;
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0xFFFF << 5;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1

    HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  printf("Error\r\n");
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
