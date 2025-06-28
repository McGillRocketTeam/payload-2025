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

#define ADC_RAW_TO_ACCELERATION(raw, gain) ((ADC_RAW_TO_VOLTAGE(raw) - ADC_VOLTAGE_HALF) / gain)
#define NORMALIZED_AMPLITUDE(a, b) (sqrtf(a * a + b * b) / FFT_SIZE_SINGLE)
#define INDEX_TO_FREQUENCY(i) ((float)i * SAMPLE_RATE_HZ / FFT_SIZE_SINGLE)

void analyze_axis(arm_rfft_fast_instance_f32 *fft_handler, volatile uint16_t fft_buffer[FFT_SIZE_SINGLE], float32_t amplitudes[FFT_AMPLITUDE_SIZE], float gain);

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

    // Analyze each axis
    analyze_axis(&accel->fft_handler, accel->fft_buffer_x, accel->amplitudes_x, ACCELEROMETER_GAIN_X);
    analyze_axis(&accel->fft_handler, accel->fft_buffer_y, accel->amplitudes_y, ACCELEROMETER_GAIN_Y);
    analyze_axis(&accel->fft_handler, accel->fft_buffer_z, accel->amplitudes_z, ACCELEROMETER_GAIN_Z);

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

void analyze_axis(arm_rfft_fast_instance_f32 *fft_handler, volatile uint16_t fft_buffer[FFT_SIZE_SINGLE], float32_t amplitudes[FFT_AMPLITUDE_SIZE], float gain)
{
    // Calculate voltage readings based on raw ADC values
    float32_t voltages[FFT_SIZE_SINGLE] = {0};
    for (int i = 0; i < FFT_SIZE_SINGLE; i++)
    {
        voltages[i] = ADC_RAW_TO_ACCELERATION(fft_buffer[i], gain);
    }

    // Perform FFT and store in output buffers
    float32_t fft_outputs[FFT_SIZE_SINGLE] = {0};
    // The final zero indicates were are not performing an inverse FFT
    arm_rfft_fast_f32(fft_handler, voltages, fft_outputs, 0);

    // Calculate DC frequency amplitude
    amplitudes[0] = NORMALIZED_AMPLITUDE(fft_outputs[0], fft_outputs[1]);
    // Calculate other amplitudes
    for (int i = 2; i < FFT_SIZE_SINGLE; i += 2)
    {
        amplitudes[i / 2] = 2 * NORMALIZED_AMPLITUDE(fft_outputs[i], fft_outputs[i + 1]);
    }
}
