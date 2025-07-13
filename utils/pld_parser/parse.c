#include "parse.h"
#include <stdio.h>
#include <string.h>
#include <libgen.h>

#define READ_ERROR(what)                                                        \
    {                                                                           \
        if (!feof(file_data))                                                   \
        {                                                                       \
            fprintf(stderr, "Error reading " what " from file: %s\n", argv[i]); \
        }                                                                       \
        break;                                                                  \
    }

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        fprintf(stderr, "Usage: %s input_file...\n", argv[0]);
        return 1;
    }

    // Process each input file
    for (int i = 1; i < argc; i++)
    {
        char *name = basename(argv[i]);

        // Ensure the file name is valid
        char file_prefix[32];
        // Find extension location
        int extension_location = strlen(name) - (sizeof(FILE_NAME_EXTENSION) - 1);
        // Check if the file name ends with the expected extension
        if (0 == strncmp(name + (extension_location),
                         FILE_NAME_EXTENSION,
                         sizeof(FILE_NAME_EXTENSION) - 1))
        {
            // Set the file prefix to the name without the extension
            int prefix_size = sizeof(file_prefix) - 1;
            // Ensure we do not overflow the prefix size by choosing the minimum
            strncpy(file_prefix, name, prefix_size < extension_location ? prefix_size : extension_location);
        }
        else
        {
            fprintf(stderr, "Invalid file format: %s\n", argv[i]);
            continue;
        }

        // Open file
        FILE *file_data = fopen(argv[i], "r");
        if (!file_data)
        {
            fprintf(stderr, "Error opening file: %s\n", argv[i]);
            continue;
        }

        // Create CSV files
        char csv_filename[128];
        char *dir = dirname(argv[i]);
        // Accelerometer CSV file
        snprintf(csv_filename, sizeof(csv_filename), "%s/" FILE_NAME_FORMAT_ACCELEROMETER, dir, file_prefix);
        FILE *csv_accel = fopen(csv_filename, "w+");
        // Telemetry CSV file
        snprintf(csv_filename, sizeof(csv_filename), "%s/" FILE_NAME_FORMAT_TELEMETRY, dir, file_prefix);
        FILE *csv_telemetry = fopen(csv_filename, "w+");

        // Write CSV headers
        if (fprintf(csv_accel, CSV_HEADER_ACCELEROMETER) < 0 ||
            fprintf(csv_telemetry, CSV_HEADER_TELEMETRY) < 0)
        {
            fprintf(stderr, "Error writing header to CSV files for %s\n", argv[i]);
            fclose(file_data);
            fclose(csv_accel);
            fclose(csv_telemetry);
            continue;
        }

        // Track total number of valid bytes read
        int bytes_valid = 0;

        while (!feof(file_data))
        {
            // Read packet header
            char header[PACKET_HEADER_LENGTH];
            if (1 != fread(header, PACKET_HEADER_LENGTH, 1, file_data))
                READ_ERROR("header")

            // Accelerometer packet
            if (strncmp(header, HEADER_ACCELEROMETER, PACKET_HEADER_LENGTH) == 0)
            {
                // Header valid
                bytes_valid += PACKET_HEADER_LENGTH;
                // Read accelerometer packet from file
                SD_packet_accelerometer packet;
                if (1 != fread(&packet, sizeof(SD_packet_accelerometer), 1, file_data))
                    READ_ERROR("accelerometer packet")

                bytes_valid += sizeof(SD_packet_accelerometer);
                // Write to CSV file
                // Extrapolate data in buffers backwards based on sampling rate
                for (int j = 0; j < FFT_SIZE_SINGLE; j++)
                {
                    if (fprintf(csv_accel, "%f,%d,%d,%d\n",
                                packet.time_elapsed +
                                    // TODO: Investigate the time extrapolation
                                    ((j - (FFT_SIZE_SINGLE - 1)) * (1 / SAMPLING_RATE * 1000)),
                                packet.x_buffer[j],
                                packet.y_buffer[j],
                                packet.z_buffer[j]) < 0)
                    {
                        fprintf(stderr, "Error writing accelerometer data to CSV file for file: %s\n", argv[i]);
                        break;
                    }
                }
            }
            // Telemetry packet
            else if (strncmp(header, HEADER_TELEMETRY, PACKET_HEADER_LENGTH) == 0)
            {
                // Header valid
                bytes_valid += PACKET_HEADER_LENGTH;
                // Read telemetry packet from file
                SD_packet_telemetry packet;
                if (1 != fread(&packet, sizeof(SD_packet_telemetry), 1, file_data))
                    READ_ERROR("telemetry packet")

                bytes_valid += sizeof(SD_packet_telemetry);
                // Write to CSV file
                if (fprintf(csv_telemetry, "%u,%d,%d,%d,%f,%f,%f,%f,%f\n",
                            packet.time_elapsed,
                            packet.ok,
                            packet.sampling_state,
                            packet.temp_control_state,
                            packet.target_temp,
                            packet.current_temp,
                            packet.current_pressure,
                            packet.current_humidity,
                            packet.battery_voltage) < 0)
                {
                    fprintf(stderr, "Error writing telemetry data to CSV file for file: %s\n", argv[i]);
                    break;
                }
            }
            // Corrupted packet
            else
            {
                // Move forward one character and attempt to read the next header
                // We do this by going backwards by one less than the length of what we just read
                fseek(file_data, -(PACKET_HEADER_LENGTH - 1), SEEK_CUR);
            }
        }
        // Ensure we reached the end of the file in case of a reading error, for determining total size
        fseek(file_data, 0, SEEK_END);
        // Format total file size into a string to find out how long it is (for padding)
        long file_size = ftell(file_data);
        char bytes_valid_buffer[32];
        snprintf(bytes_valid_buffer, sizeof(bytes_valid_buffer), "%ld", file_size);
        float bytes_valid_percentage = (float)bytes_valid / file_size * 100;
        // Decide colors
        char *color;
        if (bytes_valid_percentage > 97.5f)
        {
            color = COLOR_BRIGHT_GREEN;
        }
        else if (bytes_valid_percentage > 95.0f)
        {
            color = COLOR_BRIGHT_YELLOW;
        }
        else
        {
            color = COLOR_BRIGHT_RED;
        }
        printf("Finished file %s: \t%s%5.1f%%" COLOR_RESET " \t(%*d/%s bytes) valid\n",
               argv[i],
               color,
               bytes_valid_percentage,
               (int)strlen(bytes_valid_buffer),
               bytes_valid,
               bytes_valid_buffer);

        // Close files
        fclose(file_data);
        fclose(csv_accel);
        fclose(csv_telemetry);
    }

    return 0;
}
