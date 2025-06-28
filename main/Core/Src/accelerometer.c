/*
 * accelerometer.c
 *
 *  Created on: Jun 8, 2025
 *      Author: akash
 */

#include "accelerometer.h"
/*
 * Note:
 * ARM math header needs to be included after accelerometer header,
 * since the accelerometer header includes the STM32 drivers which
 * contain information about the MCU which the ARM math library needs.
 */
//
#include "arm_math.h"
#include "ADC.h"
#include "serial_monitor.h"

#define ADC_RAW_TO_ACCELERATION(raw) (ADC_RAW_TO_VOLTAGE(raw) - ADC_VOLTAGE_HALF)
#define NORMALIZED_AMPLITUDE(a, b) (sqrtf(a * a + b * b) / FFT_SIZE_SINGLE)
#define INDEX_TO_FREQUENCY(i) ((float)i * SAMPLE_RATE_HZ / FFT_SIZE_SINGLE)

bool PL_Accelerometer_Init(PL_Accelerometer_Handler *accel, TIM_HandleTypeDef *timer, GPIO_TypeDef *power_GPIO_Port, uint16_t power_Pin)
{
    accel->timer = timer;
    accel->power_GPIO_Port = power_GPIO_Port;
    accel->power_Pin = power_Pin;
    accel->analysis_ready = false;
    return arm_rfft_fast_init_f32(&accel->fft_handler, FFT_SIZE_SINGLE) == ARM_MATH_SUCCESS;
}

bool PL_Accelerometer_Start(PL_Accelerometer_Handler *accel)
{
    // Power on the accelerometer
    HAL_GPIO_WritePin(accel->power_GPIO_Port, accel->power_Pin, GPIO_PIN_SET);
    // Start the timer which triggers the accelerometer ADC conversions
    return HAL_TIM_Base_Start(accel->timer) == HAL_OK;
}

bool PL_Accelerometer_Stop(PL_Accelerometer_Handler *accel)
{
    // Power off the accelerometer
    HAL_GPIO_WritePin(accel->power_GPIO_Port, accel->power_Pin, GPIO_PIN_RESET);
    // Stop the timer which triggers the accelerometer ADC conversions
    return HAL_TIM_Base_Stop(accel->timer) == HAL_OK;
}

void PL_Accelerometer_Record(PL_Accelerometer_Handler *accel, volatile uint16_t *buffer)
{
    // Copy the data from the buffer to the FFT buffers
    for (int i = 0; i < FFT_SIZE_TRIPLE; i += 3)
    {
        accel->fft_buffer_x[i / 3] = buffer[i];
        accel->fft_buffer_y[i / 3] = buffer[i + 1];
        accel->fft_buffer_z[i / 3] = buffer[i + 2];
    }
    // Flag that the analysis is ready
    accel->analysis_ready = true;
}

void PL_Accelerometer_Analyze(PL_Accelerometer_Handler *accel)
{
    if (!accel->analysis_ready)
    {
        return; // Analysis not ready
    }

    // Calculate voltage readings based on raw ADC values
    float32_t voltage_x[FFT_SIZE_SINGLE] = {0};
    float32_t voltage_y[FFT_SIZE_SINGLE] = {0};
    float32_t voltage_z[FFT_SIZE_SINGLE] = {0};
    for (int i = 0; i < FFT_SIZE_SINGLE; i++)
    {
        voltage_x[i] = ADC_RAW_TO_ACCELERATION(accel->fft_buffer_x[i]);
        voltage_y[i] = ADC_RAW_TO_ACCELERATION(accel->fft_buffer_y[i]);
        voltage_z[i] = ADC_RAW_TO_ACCELERATION(accel->fft_buffer_z[i]);
    }

    // Perform FFT and store in output buffers
    float32_t fft_output_x[FFT_SIZE_SINGLE] = {0};
    float32_t fft_output_y[FFT_SIZE_SINGLE] = {0};
    float32_t fft_output_z[FFT_SIZE_SINGLE] = {0};
    arm_rfft_fast_f32(&accel->fft_handler, voltage_x, fft_output_x, 0);
    arm_rfft_fast_f32(&accel->fft_handler, voltage_y, fft_output_y, 0);
    arm_rfft_fast_f32(&accel->fft_handler, voltage_z, fft_output_z, 0);

    // Calculate DC frequency amplitude
    accel->amplitudes_x[0] = NORMALIZED_AMPLITUDE(fft_output_x[0], fft_output_x[1]);
    accel->amplitudes_y[0] = NORMALIZED_AMPLITUDE(fft_output_y[0], fft_output_y[1]);
    accel->amplitudes_z[0] = NORMALIZED_AMPLITUDE(fft_output_z[0], fft_output_z[1]);
    // Calculate other amplitudes
    for (int i = 2; i < FFT_SIZE_SINGLE; i += 2)
    {
        accel->amplitudes_x[i / 2] = 2 * NORMALIZED_AMPLITUDE(fft_output_x[i], fft_output_x[i + 1]);
        accel->amplitudes_y[i / 2] = 2 * NORMALIZED_AMPLITUDE(fft_output_y[i], fft_output_y[i + 1]);
        accel->amplitudes_z[i / 2] = 2 * NORMALIZED_AMPLITUDE(fft_output_z[i], fft_output_z[i + 1]);
    }

    accel->analysis_ready = false;
}

float PL_Accelerometer_PeakFrequency(float *amplitudes, float *peak_amplitude)
{
    /*
     * We don't ignore the DC frequency.
     * This is because the hardware DC offset added in hardware is removed in `ADC_RAW_TO_ACCELERATION`.
     * If the DC frequency is the peak frequency, then something interesting is going on and we should report it.
     */
    int peak_index = 0;
    for (int i = 0; i < FFT_AMPLITUDE_SIZE; i++)
    {
        if (amplitudes[i] > amplitudes[peak_index])
        {
            peak_index = i;
        }
    }
    *peak_amplitude = amplitudes[peak_index];
    return INDEX_TO_FREQUENCY(peak_index);
}
