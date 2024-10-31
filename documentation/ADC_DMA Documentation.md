# ADC Implementation with DMA Documentation

## 1. Introduction

**ADC (Analog-to-Digital Converter)**:

This is a peripheral used in microcontrollers to convert an analog signal (voltage) into a digital value. It reads voltages within a specified range (usually between 0 V and the reference voltage) and converts them into binary values, which are easier for the CPU to process. The ADC's functionality can vary depending on the microcontroller, and it is commonly used to digitize sensor inputs or other analog signals.

**DMA (Direct Memory Access)** :

The DMA controller is a specialized peripheral used for transferring large amounts of data directly between memory and other peripherals (such as ADC, UART, etc.) without burdening the CPU. This way, the CPU doesn't need to read every byte of data and can focus on other tasks while the DMA handles the transfer. This is especially useful when transferring continuous streams of data, such as ADC readings, to memory. There is some initial overhead in setting up the DMA, so it's most beneficial when dealing with large, continuous data transfers.

In essence, DMA helps offload the CPU when dealing with large data transfers, while ADC helps convert analog signals into digital data that the microcontroller can process.

![1727580502145](image/ADC_DMADocumentation/1727580502145.png)

---

This document provides a detailed explanation of an **ADC with DMA** (Direct Memory Access) implementation used to sample three separate ADC channels concurrently on an STM32 microcontroller. The design leverages the STM32's ability to use DMA for efficient data transfer, reducing CPU load and allowing continuous data acquisition. The ADCs are used to capture amplitude and frequency data, which is then processed and stored.

In this implementation, three separate ADC instances are set up (ADC1, ADC2, ADC3) to sample analog signals representing `X`, `Y`, and `Z` axes. The data is stored in three buffers (`adc_buffer_x`, `adc_buffer_y`, and `adc_buffer_z`) using DMA. This process ensures fast, continuous data transfer without requiring CPU intervention. The DMA controller automatically transfers ADC data to the buffers, which are then processed for logging or transmission.

## 2. Demo

### 2. 1 Create a new  Project

1. File > New > STM32 Project
2. Commercial Part Number: "NUCLEO -F446ZE"
3. Select board
   1. Name your project
   2. Language: C
   3. Binary Type: Executable
   4. Project Type: STM32Cube

Now your set. It may take a while

### 2.2 IOC Setup

See Demo documentation repo [Here](https://github.com/McGillRocketTeam/payload-2024/tree/main/payload)

The details are well explained in `doc.docx`

The code implementation is in `payload > Core > Src > main.c`


## 3. V5PCB_test1 project code details (in main.c)

> Full Implementation in V5PCB_test1 project > Core > Src > main.c

**3.1 Initialization of ADCs and DMA:**
The three ADCs (`hadc1`, `hadc2`, and `hadc3`) are initialized with DMA enabled. This configuration ensures that each ADC can convert analog inputs to digital values and store the results in a dedicated buffer. The following settings are crucial for each ADC:

* **Resolution:** 12-bit for high accuracy.
* **DMA Requests:** Continuous requests are enabled so that the ADC continuously fills the DMA buffer.
* **External Trigger:** ADC conversions are triggered by a timer (`TIM2`), ensuring consistent sampling intervals.
* **Data Alignment:** Data is right-aligned to fit within the 16-bit buffer.

**3.2 Buffer Configuration:**
The ADCs store sampled data into the following buffers:

```
/* USER CODE BEGIN 0 */
//ADC Buffer
uint16_t adc_buffer_x[16+ADC_BUFFER_SIZE] = {0};
uint16_t adc_buffer_y[16+ADC_BUFFER_SIZE] = {0};
uint16_t adc_buffer_z[16+ADC_BUFFER_SIZE] = {0};
/* USER CODE END 0 */
```

The additional 16 elements in each buffer are used for storing metadata, such as timestamps. The actual ADC data starts from index 16 onwards.

**3.3 DMA Start:**
DMA transfers are started for each ADC using the following commands:

* `HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length)`
* This function starts the ADC with DMA. The peripheral then runs in the background and you can interact with it through the callbacks
* `HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc)`
  * This function is used to stop the ADC-DMA operations if necessary.

```
 /* USER CODE BEGIN 2 */
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
  status = HAL_ADC_Start_DMA(&hadc3, (uint32_t ) &adc_buffer_z[16], ADC_BUFFER_SIZE);
  if (status != HAL_OK)
  {
    myprintf("HAL_ADC_Start_DMA adc3 error (%i)\r\n", status);
    while (1);
  }
 / USER CODE END 2 */
```

These commands configure the DMA to fill the respective buffers with `ADC_BUFFER_SIZE` samples starting from index 16 (metadata stored in the first 16 elements).

**3.4 Half and Full Buffer Completion Handling:**
The DMA controller triggers two key interrupts during its operation:

* **Half Completion Interrupt:** This interrupt is triggered when the first half of the buffer is filled. The respective callback function (`HAL_ADC_ConvHalfCpltCallback`) is called for each ADC. Timestamps are stored in the metadata section of the buffer (elements 8 and 9), and the `halfcpltx`, `halfcplty`, and `halfcpltz` flags are set for further processing.
* **Full Completion Interrupt:** This interrupt is triggered when the entire buffer is filled. The `HAL_ADC_ConvCpltCallback` is called for each ADC. Again, timestamps are saved, and the `cpltx`, `cplty`, and `cpltz` flags are set for further processing.

```
/* USER CODE BEGIN 4 */
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
```

In this callback, the current time (in milliseconds) is saved to the buffer, and a flag is set to indicate that half of the buffer is ready for processing. Same idea with the full completion interrupt.

## 4. Useful links
[Getting Started with STM32 - Working with ADC and DMA](https://www.digikey.ca/en/maker/projects/getting-started-with-stm32-working-with-adc-and-dma/f5009db3a3ed4370acaf545a3370c30c)

[STM32 Timer-based ADC sampling at a specific rate, with debug output](https://youtu.be/r9UlXjrAOtc?si=GdtByExQN0a1NJmm)















