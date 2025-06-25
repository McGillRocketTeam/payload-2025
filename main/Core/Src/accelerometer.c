/*
 * accelerometer.c
 *
 *  Created on: Jun 8, 2025
 *      Author: akash
 */

#include "accelerometer.h"
#include "arm_math.h"
#include "serial_monitor.h"

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

    float32_t voltage_x[FFT_SIZE_SINGLE] = {0};
    float32_t voltage_y[FFT_SIZE_SINGLE] = {0};
    float32_t voltage_z[FFT_SIZE_SINGLE] = {0};

    float32_t fft_output_x[FFT_SIZE_SINGLE] = {0};
    float32_t fft_output_y[FFT_SIZE_SINGLE] = {0};
    float32_t fft_output_z[FFT_SIZE_SINGLE] = {0};

    for (int i = 0; i < FFT_SIZE_SINGLE; i++)
    {
        voltage_x[i] = ACCELEROMETER_HIGH_VOLTAGE * (accel->fft_buffer_x[i] / 4096.0f);
        voltage_y[i] = ACCELEROMETER_HIGH_VOLTAGE * (accel->fft_buffer_y[i] / 4096.0f);
        voltage_z[i] = ACCELEROMETER_HIGH_VOLTAGE * (accel->fft_buffer_z[i] / 4096.0f);
    }

    // gonna assume amp buffers intialized elsewhere

    arm_rfft_fast_f32(&accel->fft_handler, voltage_x, (float32_t *)fft_output_x, 0);
    arm_rfft_fast_f32(&accel->fft_handler, voltage_y, (float32_t *)fft_output_y, 0);
    arm_rfft_fast_f32(&accel->fft_handler, voltage_z, (float32_t *)fft_output_z, 0);

 //no need to analyze DC and nyquist frequencies separately since not used
    for (int i = 0; i < FFT_SIZE_SINGLE; i += 2)
    {
        accel->amplitudes_x[i / 2] = 2 * NORMALIZED_AMPLITUDE(fft_output_x[i], fft_output_x[i + 1]);
        accel->amplitudes_y[i / 2] = 2 * NORMALIZED_AMPLITUDE(fft_output_y[i], fft_output_y[i + 1]);
        accel->amplitudes_z[i / 2] = 2 * NORMALIZED_AMPLITUDE(fft_output_z[i], fft_output_z[i + 1]);
    }

    accel->analysis_ready = false;
}

float PL_Accelerometer_PeakFrequency(float *amplitudes, float *peak_amplitude)
{
    int peak_index = 1; //ignore DC offset
    for (int i = 1; i < FFT_AMPLITUDE_SIZE-1; i++)
    {
        if (amplitudes[i] > amplitudes[peak_index])
        {
            peak_index = i;
        }
    }
    *peak_amplitude = amplitudes[peak_index];
    return INDEX_TO_FREQUENCY(peak_index);
}
