/*
 * accelerometer.c
 *
 *  Created on: Jun 8, 2025
 *      Author: akash
 */

#include "accelerometer.h"
#include "arm_math.h"
#include "serial_monitor.h"

#define NORMALIZED_AMPLITUDE(a, b) ((a * a + b * b) / FFT_SIZE_SINGLE)
#define INDEX_TO_FREQUENCY(i) ((float)i * SAMPLE_RATE_HZ / FFT_SIZE_SINGLE / 2.0f)

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

    float voltage_x[FFT_SIZE_SINGLE] = {0};
    float voltage_y[FFT_SIZE_SINGLE] = {0};
    float voltage_z[FFT_SIZE_SINGLE] = {0};

    float fft_output_x[FFT_SIZE_SINGLE] = {0};
    float fft_output_y[FFT_SIZE_SINGLE] = {0};
    float fft_output_z[FFT_SIZE_SINGLE] = {0};

    for (int i = 0; i < FFT_SIZE_SINGLE; i++)
    {
        voltage_x[i] = ACCELEROMETER_HIGH_VOLTAGE * (accel->fft_buffer_x[i] / 4096.0f);
        voltage_y[i] = ACCELEROMETER_HIGH_VOLTAGE * (accel->fft_buffer_y[i] / 4096.0f);
        voltage_z[i] = ACCELEROMETER_HIGH_VOLTAGE * (accel->fft_buffer_z[i] / 4096.0f);
    }

    // gonna assume amp buffers intialized elsewhere

    uint32_t time = HAL_GetTick();
    printf("BEFORE X: %ld\r\n", HAL_GetTick() - time);
    arm_rfft_fast_f32(&accel->fft_handler, voltage_x, (float *)fft_output_x, 0);
    printf("time after x: %ld\r\n", HAL_GetTick() - time);
    time = HAL_GetTick();
    arm_rfft_fast_f32(&accel->fft_handler, voltage_y, (float *)fft_output_y, 0);
    printf("time after y: %ld\r\n", HAL_GetTick() - time);
    time = HAL_GetTick();
    arm_rfft_fast_f32(&accel->fft_handler, voltage_z, (float *)fft_output_z, 0);
    printf("time after z: %ld\r\n", HAL_GetTick() - time);
    printf("AFTER Z: %ld\r\n", HAL_GetTick() - time);

    accel->amplitudes_x[0] = NORMALIZED_AMPLITUDE(fft_output_x[0], fft_output_x[1]);
    accel->amplitudes_y[0] = NORMALIZED_AMPLITUDE(fft_output_y[0], fft_output_y[1]);
    accel->amplitudes_z[0] = NORMALIZED_AMPLITUDE(fft_output_z[0], fft_output_z[1]);

    for (int i = 2; i < FFT_SIZE_SINGLE - 2; i += 2)
    {
        accel->amplitudes_x[i / 2] = 2 * NORMALIZED_AMPLITUDE(fft_output_x[i], fft_output_x[i + 1]);
        accel->amplitudes_y[i / 2] = 2 * NORMALIZED_AMPLITUDE(fft_output_y[i], fft_output_y[i + 1]);
        accel->amplitudes_z[i / 2] = 2 * NORMALIZED_AMPLITUDE(fft_output_z[i], fft_output_z[i + 1]);
    }

    accel->amplitudes_x[FFT_SIZE_SINGLE / 2 - 1] = NORMALIZED_AMPLITUDE(fft_output_x[FFT_SIZE_SINGLE - 2], fft_output_x[FFT_SIZE_SINGLE - 1]);
    accel->amplitudes_y[FFT_SIZE_SINGLE / 2 - 1] = NORMALIZED_AMPLITUDE(fft_output_y[FFT_SIZE_SINGLE - 2], fft_output_y[FFT_SIZE_SINGLE - 1]);
    accel->amplitudes_z[FFT_SIZE_SINGLE / 2 - 1] = NORMALIZED_AMPLITUDE(fft_output_z[FFT_SIZE_SINGLE - 2], fft_output_z[FFT_SIZE_SINGLE - 1]);

    accel->analysis_ready = false;
}

float PL_Accelerometer_PeakFrequency(float *amplitudes, float *peak_amplitude)
{
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
