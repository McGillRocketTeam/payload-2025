#include "parse.h"
#include <stdio.h>
#include <string.h>
#include <libgen.h>

#define READ_ERROR(what)                                                                              \
    {                                                                                                 \
        if (!feof(file_data))                                                                         \
        {                                                                                             \
            fprintf(stderr, "\n" COLOR_RED "Error reading " what COLOR_RESET " from: %s\n", argv[i]); \
        }                                                                                             \
        break;                                                                                        \
    }

#define LIST_ACC_TIME(instance, type, name, size, i) \
    LIST(instance, type, name, size, +((i - (FFT_SIZE_SINGLE - 1)) * (1 / SAMPLING_RATE * 1000)))
#define LIST_ACC_VALUE(instance, type, name, size, i) \
    LIST(instance, type, name, size, [i])

bool write_accelerometer_data(FILE *csv_accel, SD_packet_accelerometer packet);

struct progress_bar
{
    FILE *file_data;
    long file_size;
    char *name;
    char file_size_buffer[32];
    int file_size_length;
    int progress_bar_length;
};

void print_progress(struct progress_bar bar, int bytes_valid, int accelerometer_packets, int telemetry_packets);

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        fprintf(stderr, "Usage: %s input_file...\n", argv[0]);
        return 1;
    }

    int flag_index = 0;
    bool save_accelerometer = true;
    bool save_telemetry = true;
    for (int i = 0; i < argc; i++)
    {
        if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--accelerometer") == 0)
        {
            flag_index = i;
            save_telemetry = false;
        }
        else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--telemetry") == 0)
        {
            flag_index = i;
            save_accelerometer = false;
        }
    }
    if (!save_accelerometer && !save_telemetry)
    {
        fprintf(stderr, "Cannot use both accelerometer and telemetry flags at once.\n");
        return 1;
    }

    // Process each input file
    for (int i = 1; i < argc; i++)
    {
        if (i == flag_index)
        {
            continue;
        }

        // Copy of file path to pass to `basename` and `dirname`
        // Initialize to zero to ensure termination
        char path[256] = {0};
        strncpy(path, argv[i], sizeof(path) - 1);
        // Get base and directory names
        char *name = basename(path);
        char *dir = dirname(path);

        // Ensure the file name is valid
        // Initialize to zero to ensure termination
        char file_prefix[32] = {0};
        // Find extension location
        int extension_location = strlen(name) - (sizeof(FILE_NAME_EXTENSION) - 1);
        // Check if the file name ends with the expected extension
        if (0 == strncasecmp(name + (extension_location),
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
            fprintf(stderr, COLOR_RED "Invalid file format" COLOR_RESET ": %s\n", argv[i]);
            continue;
        }

        // Open file
        FILE *file_data = fopen(argv[i], "rb");
        if (!file_data)
        {
            fprintf(stderr, COLOR_RED "Error opening file" COLOR_RESET ": %s\n", argv[i]);
            continue;
        }

        // Create CSV files
        char csv_filename[256];
        // Accelerometer CSV file
        snprintf(
            csv_filename,
            sizeof(csv_filename),
            "%s/" FILE_NAME_FORMAT_ACCELEROMETER,
            dir,
            file_prefix);
        FILE *csv_accel;
        if (save_accelerometer)
        {
            csv_accel = fopen(csv_filename, "w+");
        }
        // Telemetry CSV file
        snprintf(
            csv_filename,
            sizeof(csv_filename),
            "%s/" FILE_NAME_FORMAT_TELEMETRY,
            dir,
            file_prefix);
        FILE *csv_telemetry;
        if (save_telemetry)
        {
            csv_telemetry = fopen(csv_filename, "w+");
        }

        // Write CSV headers
        bool accel_result = save_accelerometer ? fprintf(csv_accel, CSV_HEADER_ACCELEROMETER) >= 0 : true;
        bool telemetry_result = save_telemetry ? fprintf(csv_telemetry, CSV_HEADER_TELEMETRY) >= 0 : true;
        if (!accel_result || !telemetry_result)
        {
            fprintf(
                stderr,
                COLOR_RED "Error writing header to CSV files" COLOR_RESET
                          " for: %s\n",
                argv[i]);
            fclose(file_data);
            if (save_accelerometer)
            {
                fclose(csv_accel);
            }
            if (save_telemetry)
            {
                fclose(csv_telemetry);
            }
            continue;
        }

        // Determine total file size by moving to the end of the file, finding the byte index of the location,
        // then moving back to the beginning.
        fseek(file_data, 0, SEEK_END);
        const long file_size = ftell(file_data);
        fseek(file_data, 0, SEEK_SET);

        struct progress_bar bar = {0};
        bar.file_data = file_data;
        bar.file_size = file_size;
        bar.name = name;
        // Format total file size into a string to find out how long it is (for padding)
        snprintf(bar.file_size_buffer, sizeof(bar.file_size_buffer), "%ld", file_size);
        bar.file_size_length = strlen(bar.file_size_buffer);
        const int file_name_length = strlen(name);
        bar.progress_bar_length = PROGRESS_BAR_LENGTH_MIN > file_name_length
                                      ? PROGRESS_BAR_LENGTH_MIN
                                      : file_name_length;

        // If the file is empty
        if (file_size == 0)
        {
            printf(
                "[" COLOR_BACKGROUND_WHITE
                "%-*s" COLOR_RESET
                "] " COLOR_DARK_GRAY "Empty" COLOR_RESET "\n",
                bar.progress_bar_length, name);
            continue;
        }

        // Track total number of valid bytes read
        int bytes_valid = 0;
        int accelerometer_packets = 0;
        int telemetry_packets = 0;

        print_progress(bar, bytes_valid, accelerometer_packets, telemetry_packets);

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
                if (save_accelerometer)
                {
                    // Extrapolate data in buffers backwards based on sampling rate
                    if (!write_accelerometer_data(csv_accel, packet))
                    {
                        fprintf(
                            stderr,
                            "\n" COLOR_RED "Error writing accelerometer data to CSV file" COLOR_RESET
                            " for: %s\n",
                            argv[i]);
                        break;
                    }
                }
                accelerometer_packets++;
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
                if (save_telemetry)
                {
                    if (fprintf(csv_telemetry, CSV_FORMAT_TELEMETRY
                                                   TELEMETRY_PACKET(packet, LIST)) < 0)
                    {
                        fprintf(
                            stderr,
                            "\n" COLOR_RED "Error writing telemetry data to CSV file" COLOR_RESET
                            " for: %s\n",
                            argv[i]);
                        break;
                    }
                }
                telemetry_packets++;
            }
            // Corrupted packet
            else
            {
                // Move forward one character and attempt to read the next header
                // We do this by going backwards by one less than the length of what we just read
                fseek(file_data, -(PACKET_HEADER_LENGTH - 1), SEEK_CUR);
            }

            print_progress(bar, bytes_valid, accelerometer_packets, telemetry_packets);
        }
        printf("\n");

        // Close files
        fclose(file_data);
        if (save_accelerometer)
        {
            fclose(csv_accel);
        }
        if (save_telemetry)
        {
            fclose(csv_telemetry);
        }
    }

    return 0;
}

bool write_accelerometer_data(FILE *csv_accel, SD_packet_accelerometer packet)
{
    // Extrapolate data in buffers backwards based on sampling rate
    for (int i = 0; i < FFT_SIZE_SINGLE; i++)
    {
        if (fprintf(
                csv_accel,
                CSV_FORMAT_ACCELEROMETER
                    ACCELEROMETER_PACKET(packet, LIST_ACC_TIME, LIST_ACC_VALUE, i)) < 0)
        {
            return false;
        }
    }
    return true;
}

void print_progress(struct progress_bar bar, int bytes_valid, int accelerometer_packets, int telemetry_packets)
{
    long location = ftell(bar.file_data);
    int progress = location / (bar.file_size / bar.progress_bar_length);
    float bytes_valid_percentage = (float)bytes_valid / location * 100;
    // Decide colors
    char *percent_valid_color;
    if (bytes_valid_percentage > 97.5f)
    {
        percent_valid_color = COLOR_BRIGHT_GREEN;
    }
    else if (bytes_valid_percentage > 95.0f)
    {
        percent_valid_color = COLOR_BRIGHT_YELLOW;
    }
    else
    {
        percent_valid_color = COLOR_BRIGHT_RED;
    }
    char *bytes_color = location != bar.file_size ? COLOR_CYAN : COLOR_MAGENTA;

    printf(
        // Carriage return to beginning of the line
        "\r"
        // Turn off cursor to prevent flickering
        "\e[?25l"
        // Print first portion of padded filename highlighted, then second portion unhighlighted
        "[" COLOR_BACKGROUND_WHITE "%-*.*s" COLOR_RESET "%-*s"
        "] "
        "%s"
        "%*ld" COLOR_RESET "/" COLOR_MAGENTA "%s" COLOR_RESET " bytes"
        " | "
        "%s"
        "%5.1f%%" COLOR_RESET " valid "
        "(" COLOR_RED "%ld" COLOR_RESET " error bytes)"
        " | " COLOR_YELLOW "%5d" COLOR_RESET " accelerometer packets"
        " | " COLOR_BLUE "%5d" COLOR_RESET " telemetry packets"
        "\e[?25h", // Turn cursor back on
        progress,
        progress,
        bar.name,
        bar.progress_bar_length - progress,
        bar.name + progress,
        bytes_color,
        bar.file_size_length,
        location,
        bar.file_size_buffer,
        percent_valid_color,
        bytes_valid_percentage,
        location - bytes_valid,
        accelerometer_packets,
        telemetry_packets);
    fflush(stdout);
}
