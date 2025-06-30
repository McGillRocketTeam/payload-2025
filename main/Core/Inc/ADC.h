/*
 * ADC.h
 *
 *  Created on: Jun 7, 2025
 *      Author: akash
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdbool.h>
#include "stm32f4xx.h"
#include "accelerometer.h"

// Internal reference voltage for ADCs. ADCs convert voltages from 0 V to `ADC_VOLTAGE_MAX` V.
#define ADC_VOLTAGE_MAX 3.3f
#define ADC_VOLTAGE_HALF (ADC_VOLTAGE_MAX / 2)
// The ADCs have 12 bits of resolution, so the maximum raw ADC value is 2^12
#define ADC_RAW_MAX 4096

#define ADC_RAW_TO_VOLTAGE(raw) (raw * ADC_VOLTAGE_MAX / ADC_RAW_MAX)

/*
 * Battery voltage is divided by 9 before being sent to ADC, so the formula is:
 * V_bat = V_ADC * 9
 */
#define BATTERY_VOLTAGE_FACTOR 9
/* 
 * The integrated circuits do a very interesting conversion to cooler current, so the formula is:
 * I_cooler (in Amps) = 0.132 * V_ADC + 1.65
 */
#define COOLER_CURRENT_FACTOR 0.132
#define COOLER_CURRENT_OFFSET ADC_VOLTAGE_HALF

typedef struct
{
	ADC_HandleTypeDef *hadc1;
	ADC_HandleTypeDef *hadc2;
	ADC_HandleTypeDef *hadc3;
	TIM_HandleTypeDef *htim;
	volatile uint16_t vbat_sample;
	volatile uint16_t current_sample;
} PL_ADC_Handler;

/**
 * @brief Initializes the ADC handler and starts the ADCs in 
 * triple combined regular simultaneous + injected simultaneous mode.
 * @note Does not start any conversions, just prepares the ADCs for use.
 * @param adc Pointer to the ADC handler structure.
 * @param hadc1 Pointer to the first ADC handle.
 * @param hadc2 Pointer to the second ADC handle.
 * @param hadc3 Pointer to the third ADC handle.
 * @param htim Pointer to the timer handle for the timer which triggers the ADC injected conversions.
 * @param accelerometer_buffer Pointer to the buffer where the accelerometer data will be stored.
 * @return true if initialization is successful, false otherwise.
 */
bool PL_ADC_Init(
	PL_ADC_Handler *adc,
	ADC_HandleTypeDef *hadc1,
	ADC_HandleTypeDef *hadc2,
	ADC_HandleTypeDef *hadc3,
	TIM_HandleTypeDef *htim,
	volatile uint16_t accelerometer_buffer[ACCELEROMETER_SAMPLE_SIZE_TRIPLE]);
/**
 * @brief Stops the ADCs.
 * @param adc Pointer to the ADC handler structure.
 * @return true if deinitialization is successful, false otherwise.
 */
bool PL_ADC_Deinit(PL_ADC_Handler *adc);

/**
 * @brief Performs an injected conversion on the ADCs, sampling the battery voltage and current channels.
 * @note This function should be called after the ADCs have been initialized and started.
 * @note This function should be called in the timer interrupt callback 
 * for the timer triggering the injected conversions.
 * @param adc Pointer to the ADC handler structure.
 * @warning The ADC injected conversions must be set up to be triggered by the same timer whose interrupt
 * this function is called in.
 */
void PL_ADC_InjectedConversion(PL_ADC_Handler *adc);
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
