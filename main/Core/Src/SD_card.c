/*
 * SD_card.c
 *
 *  Created on: Jun 16, 2025
 *      Author: akash
 */

#include <stdio.h>
#include "stm32f446xx.h"
#include "SD_card.h"
#include "enabled.h"

bool PL_SDCard_Init(PL_SDCard_Handler *sd_card, FATFS *fs, FIL *file)
{
#if SD_CARD_ENABLED
    // Ensure SD card is inserted (the GPIO pin is low when the card is inserted)
    if (HAL_GPIO_ReadPin(SD_DETECT_GPIO_Port, SD_DETECT_Pin))
    {
        // SD card is not inserted
        return false;
    }

    sd_card->fs = fs;
    sd_card->file = file;

    // Mount the filesystem
    // path = "" for the root directory
    // opt = 1 to force immediate mount
    return f_mount(sd_card->fs, "", 1) == FR_OK;
#else
    return true;
#endif
}

bool PL_SDCard_Open(PL_SDCard_Handler *sd_card)
{
#if SD_CARD_ENABLED
    // File name buffer needs one extra byte for the null terminator
    char name[SD_FILE_NAME_MAX_LENGTH + 1] = {0};

    // Check for the lowest available file name
    int i = 0;
    int len;
    FRESULT exists;
    FILINFO info;
    do
    {
        len = snprintf(name, sizeof(name), "%s%d.%s", SD_FILE_BASE_NAME, i, SD_FILE_EXTENSION);
        exists = f_stat(name, &info);
        i++;
    } while (exists == FR_OK);
    // Fail if the file name exceeds the maximum length
    if (len > SD_FILE_NAME_MAX_LENGTH)
    {
        return false;
    }

    // Create and open the file, return success status
    return f_open(sd_card->file, name, FA_CREATE_NEW | FA_WRITE) == FR_OK;
#else
    return true;
#endif
}

bool PL_SDCard_Close(PL_SDCard_Handler *sd_card)
{
#if SD_CARD_ENABLED
    return f_close(sd_card->file) == FR_OK;
#else
    return true;
#endif
}

/*
packets sent to avionics: we also write this to the SD card

everything that starts with pl_ we record to the sd card
EXCEPT VIBRATION!

two types of packets written to SD card
character prefix to determine the type of packet
fixed amount of data (we know this from the prefix)
normal packet:

make two structures:
normal packet telemetry
raw ADC data

each of them should have the time they were recorded

two types of write functions, one for each type of packet
takes in all relevant data as parameters / struct
write to file
check number of bytes written
#if # bytes written > SD_FLUSH_BYTES
    flush the file

takes a ton of params
packs into struct
sends struct

take a look at last year's SD card drivers for inspiration/help
*/

bool PL_SDCard_WriteData_Normal(
    PL_SDCard_Handler *sd_card,
    bool ok,
    bool sampling_state,
    uint32_t time_elapsed,
    bool temp_control_state,
    uint8_t target_temp,
    uint16_t current_temp,
    uint16_t current_pressure,
    uint16_t current_humidity,
    uint8_t battery_voltage)
{
    // pack into struct
    normal_msg msg = {
        .ok = ok,
        .sampling_state = sampling_state,
        .time_elapsed = time_elapsed,
        .temp_control_state = temp_control_state,
        .target_temp = target_temp,
        .current_temp = current_temp,
        .current_pressure = current_pressure,
        .current_humidity = current_humidity,
        .battery_voltage = battery_voltage,
    };

    // write to file
    UINT bytes_written;
    sd_current_bytes_written += sizeof(msg);
    FRESULT res = f_write(sd_card->file, &msg, sizeof(msg), &bytes_written);
    if (res != FR_OK || bytes_written != sizeof(msg))
    {
        return false; // Error writing to file, or didn't write the expected number of bytes
    }

    // check number of bytes written
    if (sd_current_bytes_written >= SD_FLUSH_BYTES)
    {
        // flush the file
        res = f_sync(sd_card->file);
        if (res != FR_OK)
        {
            return false; // Error flushing file
        }
        sd_current_bytes_written = 0; // Reset the byte counter after flushing
    }

    return true; // Successfully wrote data
}

bool PL_SDCard_WriteData_RawADC(
    PL_SDCard_Handler *sd_card,
    uint16_t *x_buffer,
    uint16_t *y_buffer,
    uint16_t *z_buffer)
{
    // pack into struct
    RawADC_msg msg = {
        .x_buffer = x_buffer,
        .y_buffer = y_buffer,
        .z_buffer = z_buffer,
    };

    // write to file
    UINT bytes_written;
    sd_current_bytes_written += sizeof(msg);
    FRESULT res = f_write(sd_card->file, &msg, sizeof(msg), &bytes_written);
    if (res != FR_OK || bytes_written != sizeof(msg))
    {
        return false; // Error writing to file, or didn't write the expected number of bytes
    }

    // check number of bytes written
    if (sd_current_bytes_written >= SD_FLUSH_BYTES)
    {
        // flush the file
        res = f_sync(sd_card->file);
        if (res != FR_OK)
        {
            return false; // Error flushing file
        }
        sd_current_bytes_written = 0; // Reset the byte counter after flushing
    }

    return true; // Successfully wrote data
}