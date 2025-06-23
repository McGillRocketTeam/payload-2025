/*
 * ADC.h
 *
 *  Created on: Jun 7, 2025
 *      Author: akash
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "stm32f4xx.h"
#include <stdbool.h>

typedef struct
{
	ADC_HandleTypeDef *hadc1;
	ADC_HandleTypeDef *hadc2;
	ADC_HandleTypeDef *hadc3;
	volatile uint16_t vbat_sample;
	volatile uint16_t current_sample;
} PL_ADC_Handler;

/**
 * @brief Initializes the ADC handler and starts the ADCs in triple combined regular simultaneous + injected simultaneous mode.
 * @note Does not start any conversions, just prepares the ADCs for use.
 * @param adc Pointer to the ADC handler structure.
 * @param hadc1 Pointer to the first ADC handle.
 * @param hadc2 Pointer to the second ADC handle.
 * @param hadc3 Pointer to the third ADC handle.
 * @param accelerometer_buffer Pointer to the buffer where the accelerometer data will be stored. Should be of size `ACCELEROMETER_SAMPLE_SIZE_TRIPLE`.
 * @return true if initialization is successful, false otherwise.
 */
bool PL_ADC_Init(PL_ADC_Handler *adc, ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2, ADC_HandleTypeDef *hadc3, volatile uint16_t *accelerometer_buffer);
/**
 * @brief Stops the ADCs.
 * @param adc Pointer to the ADC handler structure.
 * @return true if deinitialization is successful, false otherwise.
 */
bool PL_ADC_Deinit(PL_ADC_Handler *adc);

/**
 * @brief Performs an injected conversion on the ADCs, sampling the battery voltage and current channels.
 * @note This function should be called after the ADCs have been initialized and started.
 * @note This function should be called in the timer interrupt callback or in a periodic task to sample the battery voltage and current.
 * @param adc Pointer to the ADC handler structure.
 * @return true if the injected conversion is successful, false otherwise.
 */
bool PL_ADC_InjectedConversion(PL_ADC_Handler *adc);
/**
 * @brief Calculated the battery voltage based off of the last sampled value from the ADC.
 * @param adc Pointer to the ADC handler structure.
 * @return The battery voltage in volts.
 */
float PL_ADC_GetBatteryVoltage(PL_ADC_Handler *adc);
/**
 * @brief Calculates the cooler current based off of the last sampled value from the ADC.
 * @param adc Pointer to the ADC handler structure.
 * @return The cooler current in amps.
 */
float PL_ADC_GetCoolerCurrent(PL_ADC_Handler *adc);

#endif /* INC_ADC_H_ */
