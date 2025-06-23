/*
 * ADC.c
 *
 *  Created on: Jun 8, 2025
 *      Author: akash
 */

#include "ADC.h"
#include "accelerometer.h"
#include "enabled.h"

bool PL_ADC_Init(PL_ADC_Handler *adc, ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2, ADC_HandleTypeDef *hadc3, volatile uint16_t *accelerometer_buffer)
{
    adc->hadc1 = hadc1;
    adc->hadc2 = hadc2;
    adc->hadc3 = hadc3;
    adc->vbat_sample = 0;
    adc->current_sample = 0;
    
    // Start ADCs
    // Starting ADC1 in triple synchronous multimode with DMA takes the values from the other two ADCs 
    HAL_StatusTypeDef status2 = HAL_ADC_Start(adc->hadc2);
    HAL_StatusTypeDef status3 = HAL_ADC_Start(adc->hadc3);
    HAL_StatusTypeDef status1 = HAL_ADCEx_MultiModeStart_DMA(hadc1, (uint32_t *) accelerometer_buffer, ACCELEROMETER_SAMPLE_SIZE_TRIPLE);
    return status1 == HAL_OK && status2 == HAL_OK && status3 == HAL_OK;
}

bool PL_ADC_Deinit(PL_ADC_Handler *adc)
{
    HAL_StatusTypeDef status2 = HAL_ADC_Stop(adc->hadc2);
    HAL_StatusTypeDef status3 = HAL_ADC_Stop(adc->hadc3);
    HAL_StatusTypeDef status1 = HAL_ADCEx_MultiModeStop_DMA(adc->hadc1);
    return status1 == HAL_OK && status2 == HAL_OK && status3 == HAL_OK;
}

bool PL_ADC_InjectedConversion(PL_ADC_Handler *adc)
{
    // Start simultaneous injected conversion on both ADCs
    HAL_StatusTypeDef start_status = HAL_ADCEx_InjectedStart(adc->hadc1);
    if (start_status != HAL_OK)
    {
        // Failed to start injected conversion
        // Return early to avoid making `HAL_ADCEx_InjectedPollForConversion` freeze
        return false;
    }
    // ADC1 injected conversion for battery voltage
    HAL_StatusTypeDef poll_status1 = HAL_ADCEx_InjectedPollForConversion(adc->hadc1, HAL_MAX_DELAY);
    adc->vbat_sample = (uint16_t) HAL_ADCEx_InjectedGetValue(adc->hadc1, ADC_INJECTED_RANK_1);
    // ADC2 injected conversion for current
    HAL_StatusTypeDef poll_status2 = HAL_ADCEx_InjectedPollForConversion(adc->hadc2, HAL_MAX_DELAY);
    adc->current_sample = (uint16_t) HAL_ADCEx_InjectedGetValue(adc->hadc2, ADC_INJECTED_RANK_1);
    return poll_status1 == HAL_OK && poll_status2 == HAL_OK;
}

float PL_ADC_GetBatteryVoltage(PL_ADC_Handler *adc)
{
    // TODO: Implement the actual voltage calculation based on the ADC value
    // Maybe look into __HAL_ADC_CALC_VREFANALOG_VOLTAGE or __HAL_ADC_CALC_DATA_TO_VOLTAGE
    // https://community.st.com/t5/stm32-mcus/how-to-use-the-stm32-adc-s-internal-reference-voltage/ta-p/621425
    // Preliminary formula: Vbat = (Vref * adc_value) / ADC_MAX_VALUE
    return (adc->vbat_sample * 3.3f) / 4096.0f; // Assuming Vref is 3.3V and ADC resolution is 12 bits
}

float PL_ADC_GetCoolerCurrent(PL_ADC_Handler *adc)
{
    // TODO: Implement the actual current calculation based on the ADC value
    return (float) adc->current_sample;
}
