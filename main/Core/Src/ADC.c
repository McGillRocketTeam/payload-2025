/*
 * ADC.c
 *
 *  Created on: Jun 8, 2025
 *      Author: akash
 */

#include "ADC.h"
#include "accelerometer.h"
#include "enabled.h"

/* 
 * To get more accurate readings, maybe consider measuring the STM32's internal reference voltage `Vrefint` channel.
 * Resource: https://community.st.com/t5/stm32-mcus/how-to-use-the-stm32-adc-s-internal-reference-voltage/ta-p/621425
 */

bool PL_ADC_Init(
	PL_ADC_Handler *adc,
	ADC_HandleTypeDef *hadc1,
	ADC_HandleTypeDef *hadc2,
	ADC_HandleTypeDef *hadc3,
	TIM_HandleTypeDef *htim,
	volatile uint16_t accelerometer_buffer[ACCELEROMETER_SAMPLE_SIZE_TRIPLE])
{
    // Initialize struct
    adc->hadc1 = hadc1;
    adc->hadc2 = hadc2;
    adc->hadc3 = hadc3;
    adc->htim = htim;
    adc->vbat_sample = 0;
    adc->current_sample = 0;
    
    // Start ADCs
    // Starting ADC1 in triple synchronous multimode with DMA takes the values from the other two ADCs 
    HAL_StatusTypeDef adc2_status = HAL_ADC_Start(hadc2);
    HAL_StatusTypeDef adc3_status = HAL_ADC_Start(hadc3);
    HAL_StatusTypeDef adc1_status = HAL_ADCEx_MultiModeStart_DMA(hadc1, (uint32_t *) accelerometer_buffer, ACCELEROMETER_SAMPLE_SIZE_TRIPLE);

    // Start ADC injected conversion sample timer
    HAL_StatusTypeDef tim_status = HAL_TIM_Base_Start_IT(htim);

    return adc1_status == HAL_OK && adc2_status == HAL_OK && adc3_status == HAL_OK && tim_status == HAL_OK;
}

bool PL_ADC_Deinit(PL_ADC_Handler *adc)
{
    HAL_StatusTypeDef adc2_status = HAL_ADC_Stop(adc->hadc2);
    HAL_StatusTypeDef adc3_status = HAL_ADC_Stop(adc->hadc3);
    HAL_StatusTypeDef adc1_status = HAL_ADCEx_MultiModeStop_DMA(adc->hadc1);
    HAL_StatusTypeDef tim_status = HAL_TIM_Base_Stop_IT(adc->htim);
    return adc1_status == HAL_OK && adc2_status == HAL_OK && adc3_status == HAL_OK && tim_status == HAL_OK;
}

void PL_ADC_InjectedConversion(PL_ADC_Handler *adc)
{
    adc->current_sample = (uint16_t) HAL_ADCEx_InjectedGetValue(adc->hadc1, ADC_INJECTED_RANK_1);
    adc->vbat_sample = (uint16_t) HAL_ADCEx_InjectedGetValue(adc->hadc2, ADC_INJECTED_RANK_1);
}

float PL_ADC_GetBatteryVoltage(PL_ADC_Handler *adc)
{
    return ADC_RAW_TO_VOLTAGE(adc->vbat_sample) * BATTERY_VOLTAGE_FACTOR;
}

float PL_ADC_GetCoolerCurrent(PL_ADC_Handler *adc)
{
    // This formula is slightly inaccurate for low currents
    return (ADC_RAW_TO_VOLTAGE(adc->current_sample) - COOLER_CURRENT_OFFSET) * COOLER_CURRENT_FACTOR;
}
