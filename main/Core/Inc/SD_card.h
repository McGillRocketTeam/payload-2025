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
 * @return true if data was written successfully, false otherwise.
 * @note This function assumes that the file is already opened.
 */
/* TODO: Decide what format data will be written in and what the function signature should be.
 * Not everything can be logged at the same time since the ADC data can only be acquired in chunks
 * via the conversion half-complete callbacks.
 */
bool PL_SDCard_WriteData(PL_SDCard_Handler *sd_card);

#endif /* INC_SD_CARD_H_ */
