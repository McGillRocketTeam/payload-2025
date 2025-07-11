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
    float target_temp,
    float current_temp,
    float current_pressure,
    float current_humidity,
    float battery_voltage)
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
    return write_packet(sd_card, SD_PACKET_HEADER_TELEMETRY, sizeof(SD_PACKET_HEADER_TELEMETRY) - 1, (void *)&msg, sizeof(msg));
#else
    return true;
#endif
}

bool PL_SDCard_WriteAccelerometer(
    PL_SDCard_Handler *sd_card,
    uint32_t time_elapsed,
    uint16_t x_buffer[FFT_SIZE_SINGLE],
    uint16_t y_buffer[FFT_SIZE_SINGLE],
    uint16_t z_buffer[FFT_SIZE_SINGLE])
{
#if SD_CARD_ENABLED
    // Copy/pack into struct
    SD_packet_accelerometer msg;
    msg.time_elapsed = time_elapsed;
    memcpy(&msg.x_buffer, x_buffer, sizeof(msg.x_buffer));
    memcpy(&msg.y_buffer, y_buffer, sizeof(msg.y_buffer));
    memcpy(&msg.z_buffer, z_buffer, sizeof(msg.z_buffer));

    // Packet header has 1 subtracted from size to not include null terminator
    return write_packet(sd_card, SD_PACKET_HEADER_ACCELEROMETER, sizeof(SD_PACKET_HEADER_ACCELEROMETER) - 1, (void *)&msg, sizeof(msg));
#else
    return true;
#endif
}

bool write_packet(PL_SDCard_Handler *sd_card, const char *prefix, size_t size_prefix, const void *data, size_t size_data)
{
    UINT bytes_written;
    FRESULT res;

    res = f_write(&sd_card->file, (void *)prefix, size_prefix, &bytes_written);
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

bool parse_sd_binary_to_csv(
    FIL *file,
    FIL *telemetry_csv_file,
    FIL *accelerometer_csv_file,
    float sampling_rate)
{
    char header[5] = {0}; // allocate space for the header, which is 4 chars long (+ \0)
    UINT bytes_read, bytes_written;
    FRESULT res;
    UINT pos = 0; // position in the file

    // write headers for the files.
    const char *telemetry_csv_header = "time,ok,sampling_state,temp_control_state,target_temp,current_temp,current_pressure,current_humidity,battery_voltage\n";
    const char *accelerometer_csv_header = "time,x,y,z\n";
    f_write(telemetry_csv_file, telemetry_csv_header, strlen(telemetry_csv_header), &bytes_written);
    f_write(accelerometer_csv_file, accelerometer_csv_header, strlen(accelerometer_csv_header), &bytes_written);

    while (pos + 4 <= f_size(file))
    {
        // find four bytes. We want to make sure it's not corrupted.
        res = f_lseek(file, pos);
        if (res != FR_OK)
        {
            return false; // Error seeking in file
        }

        // read the 4 bytes
        res = f_read(file, header, sizeof(header) - 1, &bytes_read);
        if (res != FR_OK)
        {
            return false; // Error reading file
        }

        if (bytes_read != 4)
        {
            break; // end of file reached
        }

        header[4] = '\0'; // null-terminate the string

        // check if header is telemetry packet.
        if (memcmp(header, SD_PACKET_HEADER_TELEMETRY, 4) == 0)
        {
            // read and process telemetry data
            SD_packet_telemetry telemetry_data;
            res = f_read(file, &telemetry_data, sizeof(telemetry_data), &bytes_read);
            if (res != FR_OK || bytes_read != sizeof(telemetry_data))
            {
                return false; // Error reading telemetry data
            }

            char line[200]; // allocate a large buffer. Chose 200 arbitrarily, but it should be big enough.
            int len = snprintf(line, sizeof(line), "%lu,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                               telemetry_data.time_elapsed,       // long int
                               telemetry_data.ok,                 // 1 or 0
                               telemetry_data.sampling_state,     // 1 or 0
                               telemetry_data.temp_control_state, // 1 or 0
                               telemetry_data.target_temp,        // 3 decmimal places
                               telemetry_data.current_temp,       // 3 decimal places
                               telemetry_data.current_pressure,   // 3 decimal places
                               telemetry_data.current_humidity,   // 3 decimal places
                               telemetry_data.battery_voltage);   // 3 decimal places

            if (len < 0 || len >= sizeof(line))
            {
                return false; // Error formatting telemetry data
            }

            res = f_write(telemetry_csv_file, line, len, &bytes_written);
            if (res != FR_OK || bytes_written != len)
            {
                return false; // Error writing telemetry data to CSV
            }
            pos += 4 + sizeof(telemetry_data); // move forward by 4 bytes (header) + size of telemetry data
        }
        else if (memcmp(header, SD_PACKET_HEADER_ACCELEROMETER, 4) == 0) // check if accelerometer data
        {
            SD_packet_accelerometer accelerometer_data;
            res = f_read(file, &accelerometer_data, sizeof(accelerometer_data), &bytes_read);
            if (res != FR_OK || bytes_read != sizeof(accelerometer_data))
            {
                return false; // Error reading accelerometer data
            }

            for (int i = 0; i < FFT_SIZE_SINGLE; ++i)
            {
                float sample_time = accelerometer_data.time_elapsed - (FFT_SIZE_SINGLE - 1 - i) / sampling_rate;
                char line[100];
                int len = snprintf(line, sizeof(line), "%.3f,%u,%u,%u\n",
                                   sample_time,
                                   accelerometer_data.x_buffer[i],
                                   accelerometer_data.y_buffer[i],
                                   accelerometer_data.z_buffer[i]);

                if (len < 0 || len >= sizeof(line))
                {
                    return false; // something went wrong!
                }

                res = f_write(accelerometer_csv_file, line, len, &bytes_written);
                if (res != FR_OK || bytes_written != len)
                {
                    return false; // couldn't write accelerometer data to CSV
                }
            }

            pos += 4 + sizeof(accelerometer_data); // move position forward by 4 bytes (header) + size of accelerometer data
        }
        else
        {
            pos += 1; // move position forward by 1 byte if not recognized
        }
    }
}