#ifndef __PARSE_H
#define __PARSE_H

#include <stdint.h>
#include <stdbool.h>

#define FFT_SIZE_SINGLE (4096 / 2)
#define SAMPLING_RATE 10000.0f // Hz

#define HEADER_ACCELEROMETER "ACC!"
#define HEADER_TELEMETRY "TEL!"
#define PACKET_HEADER_LENGTH 4

#define CSV_HEADER_ACCELEROMETER "time,x,y,z\n"
#define CSV_HEADER_TELEMETRY "time,ok,sampling_state,temp_control_state,\
target_temp,current_temp,current_pressure,current_humidity,battery_voltage\n"

#define FILE_NAME_BASE "DATA"
#define FILE_NAME_EXTENSION ".PLD"
#define FILE_NAME_FORMAT_DATA (FILE_NAME_BASE "%d" FILE_NAME_EXTENSION)
#define FILE_NAME_FORMAT_ACCELEROMETER "%s_accelerometer.csv"
#define FILE_NAME_FORMAT_TELEMETRY "%s_telemetry.csv"

// ANSI color escape codes
#define COLOR_RESET          "\x1b[0m"
#define COLOR_NONE           COLOR_RESET
#define COLOR_BLACK          "\x1b[30m"
#define COLOR_RED            "\x1b[31m"
#define COLOR_GREEN          "\x1b[32m"
#define COLOR_YELLOW         "\x1b[33m"
#define COLOR_BLUE           "\x1b[34m"
#define COLOR_MAGENTA        "\x1b[35m"
#define COLOR_CYAN           "\x1b[36m"
#define COLOR_LIGHT_GRAY     "\x1b[37m"
#define COLOR_DARK_GRAY      "\x1b[90m"
#define COLOR_BRIGHT_RED     "\x1b[91m"
#define COLOR_BRIGHT_GREEN   "\x1b[92m"
#define COLOR_BRIGHT_YELLOW  "\x1b[93m"
#define COLOR_BRIGHT_BLUE    "\x1b[94m"
#define COLOR_BRIGHT_MAGENTA "\x1b[95m"
#define COLOR_BRIGHT_CYAN    "\x1b[96m"
#define COLOR_WHITE          "\x1b[97m"

typedef struct __attribute__((packed))
{
    uint32_t time_elapsed;
    uint16_t x_buffer[FFT_SIZE_SINGLE];
    uint16_t y_buffer[FFT_SIZE_SINGLE];
    uint16_t z_buffer[FFT_SIZE_SINGLE];
} SD_packet_accelerometer;

typedef struct __attribute__((packed))
{
    uint32_t time_elapsed;
    bool ok;
    bool sampling_state;
    bool temp_control_state;
    float target_temp;
    float current_temp;
    float current_pressure;
    float current_humidity;
    float battery_voltage;
} SD_packet_telemetry;

#endif // __PARSE_H
