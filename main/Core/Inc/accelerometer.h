/*
 * accelerometer.h
 *
 *  Created on: Jun 7, 2025
 *      Author: akash
 */

#ifndef INC_ACCELEROMETER_H_
#define INC_ACCELEROMETER_H_

#include "stm32f4xx.h"
#include "arm_math.h"
#include <stdbool.h>

#define ACCELEROMETER_SAMPLE_SIZE_SINGLE 4096
#define ACCELEROMETER_SAMPLE_SIZE_TRIPLE (ACCELEROMETER_SAMPLE_SIZE_SINGLE * 3)
#define FFT_SIZE_SINGLE (ACCELEROMETER_SAMPLE_SIZE_SINGLE / 2)
#define FFT_SIZE_TRIPLE (FFT_SIZE_SINGLE * 3)
#define FFT_AMPLITUDE_SIZE (FFT_SIZE_SINGLE / 2)

#define SAMPLE_RATE_HZ 10000.0f

typedef struct {
	TIM_HandleTypeDef *timer;
	GPIO_TypeDef *power_GPIO_Port;
	uint16_t power_Pin;
	volatile uint16_t fft_buffer_x[FFT_SIZE_SINGLE];
	volatile uint16_t fft_buffer_y[FFT_SIZE_SINGLE];
	volatile uint16_t fft_buffer_z[FFT_SIZE_SINGLE];
	volatile bool analysis_ready;
	float amplitudes_x[FFT_AMPLITUDE_SIZE];
	float amplitudes_y[FFT_AMPLITUDE_SIZE];
	float amplitudes_z[FFT_AMPLITUDE_SIZE];
	arm_rfft_fast_instance_f32 fft_handler;
} PL_Accelerometer_Handler;

/**
 * @brief Initializes the accelerometer handler. Should be called before starting the accelerometers and after initalizing the ADCs.
 * @param accel Pointer to the accelerometer handler structure.
 * @param timer Pointer to the timer handle used for triggering ADC conversions.
 * @param power_GPIO_Port GPIO port for the accelerometer power control pin.
 * @param power_Pin GPIO pin for the accelerometer power control.
 * @note This function does not start the accelerometers or the timer, it only initializes the handler structure.
 */
bool PL_Accelerometer_Init(PL_Accelerometer_Handler *accel, TIM_HandleTypeDef *timer, GPIO_TypeDef *power_GPIO_Port, uint16_t power_Pin);
/**
 * @brief Powers on the accelerometers and starts the triple conversion trigger timer.
 * @param accel Pointer to the accelerometer handler structure.
 * @return true if the accelerometers are successfully started, false otherwise.
 */
bool PL_Accelerometer_Start(PL_Accelerometer_Handler *accel);
/**
 * @brief Powers off the accelerometers and stops the triple conversion trigger timer.
 * @param accel Pointer to the accelerometer handler structure.
 * @return true if the accelerometers are successfully stopped, false otherwise.
 */
bool PL_Accelerometer_Stop(PL_Accelerometer_Handler *accel);
/**
 * @brief Records (copies) accelerometer data as triplets into the three FFT buffers and flags analysis as ready.
 * @note This function should be called in the ADC conversion half complete/complete callback.
 * @param accel Pointer to the accelerometer handler structure.
 * @param buffer Pointer to the source buffer of raw triplet data which the ADCs write to.
 */
void PL_Accelerometer_Record(PL_Accelerometer_Handler *accel, volatile uint16_t *buffer);
/**
 * @brief Performs FFT analysis on all three axes of the recorded accelerometer data. 
 * Populates all three accelerometer amplitude buffers.
 * This function should be called after `PL_Accelerometer_Record` has been called and analysis is ready.
 * Flags analysis as not ready after completion.
 * @param accel Pointer to the accelerometer handler structure.
 * @return true if the analysis is successful, false otherwise.
 */
void PL_Accelerometer_Analyze(PL_Accelerometer_Handler *accel);
/**
 * @brief Calculates the peak frequency from the FFT amplitudes.
 * @param amplitudes Pointer to the array of FFT amplitudes. Should be of size `FFT_AMPLITUDE_SIZE`.
 * @param peak_amplitude Pointer to a float where the peak amplitude will be stored.
 * @return The peak frequency in Hz.
 */
float PL_Accelerometer_PeakFrequency(float *amplitudes, float *peak_amplitude);

#endif /* INC_ACCELEROMETER_H_ */
