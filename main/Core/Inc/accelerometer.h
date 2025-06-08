/*
 * ADC.h
 *
 *  Created on: Jun 7, 2025
 *      Author: akash
 */

#ifndef INC_ACCELEROMETER_H_
#define INC_ACCELEROMETER_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define ACCELEROMETER_SAMPLE_SIZE_SINGLE 4096
#define ACCELEROMETER_SAMPLE_SIZE_TRIPLE (ACCELEROMETER_SAMPLE_SIZE_SINGLE * 3)
#define FFT_SIZE_SINGLE (ACCELEROMETER_SAMPLE_SIZE_SINGLE / 2)
#define FFT_SIZE_TRIPLE (FFT_SIZE_SINGLE * 3)
// TODO: Verify if the +1 is correct
#define FFT_AMPLITUDE_SIZE (FFT_SIZE_SINGLE / 2 + 1)

#define SAMPLE_RATE_HZ 10000

typedef struct {
	TIM_HandleTypeDef *timer;
	GPIO_TypeDef *power_GPIO_Port;
	uint16_t power_Pin;
	uint16_t *fft_buffer_x;
	uint16_t *fft_buffer_y;
	uint16_t *fft_buffer_z;
	bool analysis_ready;
	// TODO: Add FFT instance
} PL_Accelerometer_Handler;

/**
 * @brief Initializes the accelerometer handler. Should be called before starting the accelerometers and after initalizing the ADCs.
 * @param accel Pointer to the accelerometer handler structure.
 * @param timer Pointer to the timer handle used for triggering ADC conversions.
 * @param power_GPIO_Port GPIO port for the accelerometer power control pin.
 * @param power_Pin GPIO pin for the accelerometer power control.
 * @param fft_buffer_x Pointer to the buffer which will be used for X-axis FFT data. Should be of size `FFT_SIZE_SINGLE`.
 * @param fft_buffer_y Pointer to the buffer which will be used for Y-axis FFT data. Should be of size `FFT_SIZE_SINGLE`.
 * @param fft_buffer_z Pointer to the buffer which will be used for Z-axis FFT data. Should be of size `FFT_SIZE_SINGLE`.
 * @note This function does not start the accelerometers or the timer, it only initializes the handler structure.
 */
void PL_Accelerometer_Init(PL_Accelerometer_Handler *accel, TIM_HandleTypeDef *timer, GPIO_TypeDef *power_GPIO_Port, uint16_t power_Pin, uint16_t *fft_buffer_x, uint16_t *fft_buffer_y, uint16_t *fft_buffer_z);
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
 * @brief Records (copies) accelerometer data into the three FFT buffers and flags analysis as ready.
 * @note This function should be called in the ADC conversion half complete/complete callback.
 * @param accel Pointer to the accelerometer handler structure.
 * @param buffer Pointer to the source buffer of raw triplet data which the ADCs write to.
 */
void PL_Accelerometer_Record(PL_Accelerometer_Handler *accel, uint16_t *buffer);
/**
 * @brief Performs FFT analysis on all three axes of the recorded accelerometer data.
 * This function should be called after `PL_Accelerometer_Record` has been called and analysis is ready.
 * Flags analysis as not ready after completion.
 * @param accel Pointer to the accelerometer handler structure.
 * @param amplitudes_x Pointer to the output array for X-axis FFT amplitudes. Should be of size `FFT_AMPLITUDE_SIZE`.
 * @param amplitudes_y Pointer to the output array for Y-axis FFT amplitudes. Should be of size `FFT_AMPLITUDE_SIZE`.
 * @param amplitudes_z Pointer to the output array for Z-axis FFT amplitudes. Should be of size `FFT_AMPLITUDE_SIZE`.
 * @return true if the analysis is successful, false otherwise.
 */
bool PL_Accelerometer_Analyze(PL_Accelerometer_Handler *accel, float *amplitudes_x, float *amplitudes_y, float *amplitudes_z);
/**
 * @brief Calculates the peak frequency from the FFT amplitudes.
 * @param amplitudes Pointer to the array of FFT amplitudes. Should be of size `FFT_AMPLITUDE_SIZE`.
 * @param peak_amplitude Pointer to a float where the peak amplitude will be stored.
 * @return The peak frequency in Hz.
 */
float PL_Accelerometer_PeakFrequency(float *amplitudes, float *peak_amplitude);

#endif /* INC_ACCELEROMETER_H_ */
