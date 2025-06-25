/*
 * SD_card.c
 *
 *  Created on: Jun 16, 2025
 *      Author: akash
 */

#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"
#include "SD_card.h"
#include "enabled.h"

bool write_packet(PL_SDCard_Handler *sd_card, const char *prefix, size_t size_prefix, const void *data, size_t size_data);

bool PL_SDCard_Init(PL_SDCard_Handler *sd_card)
{
#if SD_CARD_ENABLED
    // Ensure SD card is inserted (the GPIO pin is low when the card is inserted)
    if (HAL_GPIO_ReadPin(SD_DETECT_GPIO_Port, SD_DETECT_Pin))
    {
        // SD card is not inserted
        return false;
    }

    // Mount the filesystem
    // path = "" for the root directory
    // opt = 1 to force immediate mount
    return f_mount(&sd_card->fs, "", 1) == FR_OK;
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
    return f_open(&sd_card->file, name, FA_CREATE_NEW | FA_WRITE) == FR_OK;
#else
    return true;
#endif
}

bool PL_SDCard_Close(PL_SDCard_Handler *sd_card)
{
#if SD_CARD_ENABLED
    return f_close(&sd_card->file) == FR_OK;
#else
    return true;
#endif
}

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
    uint8_t battery_voltage)
{
#if SD_CARD_ENABLED
    // Pack data into struct
    SD_packet_telemetry msg = {
        .time_elapsed = time_elapsed,
        .ok = ok,
        .sampling_state = sampling_state,
        .temp_control_state = temp_control_state,
        .target_temp = target_temp,
        .current_temp = current_temp,
        .current_pressure = current_pressure,
        .current_humidity = current_humidity,
        .battery_voltage = battery_voltage,
    };

    // Packet header has 1 subtracted from size to not include null terminator
    return write_packet(sd_card, SD_PACKET_HEADER_TELEMETRY, sizeof(SD_PACKET_HEADER_TELEMETRY) - 1, (void *) &msg, sizeof(msg));
#else
    return true;
#endif
}

bool PL_SDCard_WriteAccelerometer(
    PL_SDCard_Handler *sd_card,
    uint32_t time_elapsed,
    uint16_t *x_buffer,
    uint16_t *y_buffer,
    uint16_t *z_buffer)
{
#if SD_CARD_ENABLED
    // Copy/pack into struct
    SD_packet_accelerometer msg;
    msg.time_elapsed = time_elapsed;
    memcpy(&msg.x_buffer, x_buffer, sizeof(msg.x_buffer));
    memcpy(&msg.y_buffer, y_buffer, sizeof(msg.y_buffer));
    memcpy(&msg.z_buffer, z_buffer, sizeof(msg.z_buffer));

    // Packet header has 1 subtracted from size to not include null terminator
    return write_packet(sd_card, SD_PACKET_HEADER_ACCELEROMETER, sizeof(SD_PACKET_HEADER_ACCELEROMETER) - 1, (void *) &msg, sizeof(msg));
#else
    return true;
#endif
}

bool write_packet(PL_SDCard_Handler *sd_card, const char *prefix, size_t size_prefix, const void *data, size_t size_data)
{
    UINT bytes_written;
    FRESULT res;

    res = f_write(&sd_card->file, (void *) prefix, size_prefix, &bytes_written);
    // Error writing prefix, or didn't write the expected number of bytes
    if (res != FR_OK || bytes_written != size_prefix)
    {
        return false;
    }
    sd_card->bytes_written += bytes_written;

    res = f_write(&sd_card->file, data, size_data, &bytes_written);
    // Error writing to file, or didn't write the expected number of bytes
    if (res != FR_OK || bytes_written != size_data)
    {
        return false;
    }
    sd_card->bytes_written += bytes_written;

    // Check if we need to flush the file
    if (sd_card->bytes_written >= SD_FLUSH_BYTES)
    {
        res = f_sync(&sd_card->file);
        if (res != FR_OK)
        {
            return false; // Error flushing file
        }
        sd_card->bytes_written = 0; // Reset the byte counter after flushing
    }

    return true;
}
