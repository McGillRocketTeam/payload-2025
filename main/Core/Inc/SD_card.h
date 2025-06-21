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

/**
 * Data files will be named in the format:
 * "data0.pl", "data1.pl", ..., "dataN.pl"
 */
#define SD_FILE_BASE_NAME "data"
#define SD_FILE_EXTENSION "pl"
// Flushing the file is required after writing a certain number of bytes
#define SD_FLUSH_BYTES 512
// FATFS limits file name length to 12 characters
#define SD_FILE_NAME_MAX_LENGTH 12

typedef struct
{
    FATFS *fs;
    FIL *file;
} PL_SDCard_Handler;

typedef struct __attribute__((packed))
{
    bool ok;
    bool sampling_state;
    uint32_t time_elapsed;
    bool temp_control_state;
    uint8_t target_temp;
    uint16_t current_temp;
    uint16_t current_pressure;
    uint16_t current_humidity;
    uint8_t battery_voltage;
} normal_msg;

#define ACCELEROMETER_SAMPLE_SIZE_SINGLE 256
typedef struct
{
    uint16_t x_buffer[ACCELEROMETER_SAMPLE_SIZE_SINGLE];
    uint16_t y_buffer[ACCELEROMETER_SAMPLE_SIZE_SINGLE];
    uint16_t z_buffer[ACCELEROMETER_SAMPLE_SIZE_SINGLE];
} RawADC_msg;

static uint16_t sd_current_bytes_written = 0;

/**
 * @brief Initializes the SD card handler and mounts the filesystem.
 * @param sd_card Pointer to the SD card handler structure.
 * @param fs Pointer to the FATFS structure.
 * @param file Pointer to the FIL structure.
 * @return true if initialization was successful, false otherwise.
 */
bool PL_SDCard_Init(PL_SDCard_Handler *sd_card, FATFS *fs, FIL *file);
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
 * @brief Writes data to the file on the SD card. Flushes the writes if the lines written exceeds
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
/* TODO: Decide what format data will be written in and what the function signature should be.
 * Not everything can be logged at the same time since the ADC data can only be acquired in chunks
 * via the conversion half-complete callbacks.
 */
bool PL_SDCard_WriteDataNormal(
    PL_SDCard_Handler *sd_card,
    bool ok,
    bool sampling_state,
    uint32_t time_elapsed,
    bool temp_control_state,
    uint8_t target_temp,
    uint16_t current_temp,
    uint16_t current_pressure,
    uint16_t current_humidity,
    uint8_t battery_voltage);

bool PL_SDCard_WriteData_RawADC(
    /* TODO FILL OUt*/);

#endif /* INC_SD_CARD_H_ */
