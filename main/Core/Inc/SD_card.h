/*
 * SD_card.h
 *
 *  Created on: Jun 16, 2025
 *      Author: akash
 */

#ifndef INC_SD_CARD_H_
#define INC_SD_CARD_H_

#include <stdbool.h>
#include "fatfs.h"
#include "accelerometer.h"

/**
 * Data files will be named in the format:
 * "nameN.ext"
 */
#define SD_FILE_BASE_NAME "data"
// Stands for PayLoad Data
#define SD_FILE_EXTENSION "pld"
#define SD_PACKET_HEADER_TELEMETRY "TEL"
#define SD_PACKET_HEADER_ACCELEROMETER "ACC"
// Flushing the file is required after writing a certain number of bytes
#define SD_FLUSH_BYTES 65536 // 2^16 bytes
// FATFS limits file name length 
#define SD_FILE_NAME_MAX_LENGTH 12

typedef struct
{
    FATFS fs;
    FIL file;
    UINT bytes_written;
} PL_SDCard_Handler;

typedef struct __attribute__((packed))
{
    uint32_t time_elapsed;
    bool ok;
    bool sampling_state;
    bool temp_control_state;
    uint8_t target_temp;
    uint16_t current_temp;
    uint16_t current_pressure;
    uint16_t current_humidity;
    uint8_t battery_voltage;
} SD_packet_telemetry;

typedef struct __attribute__((packed))
{
    uint32_t time_elapsed;
    uint16_t x_buffer[FFT_SIZE_SINGLE];
    uint16_t y_buffer[FFT_SIZE_SINGLE];
    uint16_t z_buffer[FFT_SIZE_SINGLE];
} SD_packet_accelerometer;

/**
 * @brief Initializes the SD card handler and mounts the filesystem.
 * @param sd_card Pointer to the SD card handler structure.
 * @param fs Pointer to the FATFS structure.
 * @param file Pointer to the FIL structure.
 * @return true if initialization was successful, false otherwise.
 */
bool PL_SDCard_Init(PL_SDCard_Handler *sd_card);
/**
 * @brief Opens a file on the SD card with a unique name.
 * @param sd_card Pointer to the SD card handler structure.
 * @return true if the file was opened successfully, false otherwise.
 */
bool PL_SDCard_Open(PL_SDCard_Handler *sd_card);
/**
 * @brief Closes the file on the SD card.
 * @param sd_card Pointer to the SD card handler structure.
 * @return true if the file was closed successfully, false otherwise.
 */
bool PL_SDCard_Close(PL_SDCard_Handler *sd_card);
/**
 * @brief Writes a telemetry packet to the file on the SD card. Flushes the writes if the lines written exceeds `SD_FLUSH_BYTES`.
 * @param sd_card Pointer to the SD card handler structure.
 * @param ok
 * @param sampling_state is the sampling on or off?
 * @param temp_control_state is the temperature control on or off?
 * @param target_temp the target temperature
 * @param current_temp the current temperature
 * @param current_pressure the current pressure
 * @param current_humidity the current humidity
 * @param battery_voltage the battery voltage
 * @return true if data was written successfully, false otherwise.
 * @note This function assumes that the file is already opened.
 */
bool PL_SDCard_WriteTelemetry(
    PL_SDCard_Handler *sd_card,
    uint32_t time_elapsed,
    bool ok,
    bool sampling_state,
    bool temp_control_state,
    uint8_t target_temp,
    uint16_t current_temp,
    uint16_t current_pressure,
    uint16_t current_humidity,
    uint8_t battery_voltage);
/**
 * @brief Writes a telemetry packet to the file on the SD card. Flushes the writes if the lines written exceeds `SD_FLUSH_BYTES`.
 * @param sd_card Pointer to the SD card handler structure.
 * @param x_buffer Pointer to buffer containing the x-axis accelerometer data. Should be of size `FFT_SIZE_SINGLE`.
 * @param y_buffer Pointer to buffer containing the y-axis accelerometer data. Should be of size `FFT_SIZE_SINGLE`.
 * @param z_buffer Pointer to buffer containing the z-axis accelerometer data. Should be of size `FFT_SIZE_SINGLE`.
 * @return true if data was written successfully, false otherwise.
 * @note This function assumes that the file is already opened.
 */
bool PL_SDCard_WriteAccelerometer(
    PL_SDCard_Handler *sd_card,
    uint32_t time_elapsed,
    uint16_t x_buffer[FFT_SIZE_SINGLE],
    uint16_t y_buffer[FFT_SIZE_SINGLE],
    uint16_t z_buffer[FFT_SIZE_SINGLE]);

#endif /* INC_SD_CARD_H_ */
