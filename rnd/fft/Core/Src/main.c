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
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FFT_SIZE 2048
#define SAMPLE_RATE 1000.0f /* Hertz */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
arm_rfft_fast_instance_f32 fft_handler;

/**
 * Buffer of sample points from signal to input to the FFT
 */
float fft_input[FFT_SIZE] = {0};
/**
 * FFT coefficients buffer.
 * Contains FFT_SIZE / 2 coefficients with each even and odd element being the
 * real and imaginary components of each coefficient, respectively.
 */
float fft_output[FFT_SIZE] = {0};
/**
 * Buffer containing the amplitudes of each coefficient.
 */
float fft_amplitudes[FFT_SIZE / 2] = {0};

int fft_index = 0;
uint8_t fft_ready = 0;

float peak_amplitude;
uint16_t peak_freq; /* Hertz */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
float normalized_amplitude(float a, float b);
uint16_t index_to_frequency(int i);
float signal_function(int t);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  arm_rfft_fast_init_f32(&fft_handler, FFT_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int counter = 0;
  while (1)
  {
	/*
	 * Get signal from source
	 * For this example, the signal is simulated from a sum of sinusoidal functions
	 * In the final code, this section would probably be put in a callback function
	 * 	triggered by the sensor sampling timer
	 */
	float signal = signal_function(counter);
	fft_input[fft_index] = signal;
	fft_index++;

	if (fft_index == FFT_SIZE)
	{
		/*
		 * Perform Fast Fourier Transform
		 * Cast the input and output buffers to (float *) to avoid a warning since
		 * we already gave the FFT instance the buffer size when instantiating it and
		 * it doesn't expect an array of a specific size.
		 */
		arm_rfft_fast_f32(&fft_handler, (float *) &fft_input, (float *) &fft_output, 0);
		// Reset FFT tracking
		fft_ready = 1;
		fft_index = 0;
	}

	/*
	 * Give a bit of visualization for the value of the signal
	 * Turn LED on when the signal is positive and off when it's negative
	 */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, signal > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	// Increase simulation step
	counter++;

	/*
	 * Handle the FFT output data
	 * This code can go in the main function as is
	 */
	if (fft_ready)
	{
		// Amplitude of zero frequency
		fft_amplitudes[0] = normalized_amplitude(fft_output[0], fft_output[1]);
		for (int i = 2; i < FFT_SIZE; i += 2)
		{
			// Amplitude of nonzero frequencies are twice the normalized amplitude of the FFT coefficients
			fft_amplitudes[i / 2] = 2 * normalized_amplitude(fft_output[i], fft_output[i + 1]);
		}

		// Find peak frequency and its amplitude
		peak_amplitude = 0;
		int peak_index = 0;
		for (int i = 0; i < FFT_SIZE / 2; i++)
		{
			if (fft_amplitudes[i] > peak_amplitude)
			{
				peak_amplitude = fft_amplitudes[i];
				peak_index = i;
			}
		}
		peak_freq = index_to_frequency(peak_index);

		// Reset FFT ready flag
		fft_ready = 0;
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// Delay 1ms to simulate sampling frequency of 1000 Hz
	HAL_Delay(1);
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * Find the amplitude of a complex number a + bi.
 * @param a: The real part of the complex number
 * @param b: The Imaginary part of the complex number.
 */
float normalized_amplitude(float a, float b)
{
	return sqrtf(a * a + b * b) / FFT_SIZE;
}

/**
 * Converts the index of the list of amplitudes to a frequency in Hz,
 * rounded to an integer.
 */
uint16_t index_to_frequency(int i)
{
	return (uint16_t) roundf(i * SAMPLE_RATE / FFT_SIZE);
}

float signal_function(int t_ms)
{
  float t_s = t_ms / SAMPLE_RATE;
  float scaled_time = t_s * (2 * PI);
  float value = cosf(scaled_time) + 2 * sinf(2 * scaled_time) + 3 * sinf(3 * scaled_time) + 4 * cosf(3 * scaled_time) + 3 * sinf(5 * scaled_time) + 0.5 * sinf(6 * scaled_time);
  return value;
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
