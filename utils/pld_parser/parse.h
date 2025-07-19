#ifndef __PARSE_H
#define __PARSE_H

#include <stdint.h>
#include <stdbool.h>

#define FFT_SIZE_SINGLE (4096 / 2)
#define SAMPLING_RATE 10000.0f // Hz

#define HEADER_ACCELEROMETER "ACC!"
#define HEADER_TELEMETRY "TEL!"
#define PACKET_HEADER_LENGTH 4

#define FILE_NAME_EXTENSION ".PLD"
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

// Macro to define a struct field
#define FIELD(instance, type, name, size, suffix) type name;
// Macro to define a struct field of array type
#define FIELD_ARRAY(instance, type, name, size, suffix) type name[size];
// Macro to list fields with suffixes and a leading comma
#define LIST(instance, type, name, size, suffix) , instance.name suffix

/**
 * @brief XMacro to loop over the fields of the accelerometer packet
 * @param instance The name of the struct instance, used when listing the fields
 * @param X The macro to be called on the elapsed time field 
 * @param Y The macro to be called on the x, y, and z buffer fields
 * @param i The index of the accelerometer buffers
 */
#define ACCELEROMETER_PACKET(instance, X, Y, i)         \
    X(instance, uint32_t, time_elapsed, , i)            \
    Y(instance, uint16_t, x_buffer, FFT_SIZE_SINGLE, i) \
    Y(instance, uint16_t, y_buffer, FFT_SIZE_SINGLE, i) \
    Y(instance, uint16_t, z_buffer, FFT_SIZE_SINGLE, i)
    
#define CSV_HEADER_ACCELEROMETER "time,x,y,z\n"
#define CSV_FORMAT_ACCELEROMETER "%f,%d,%d,%d\n"

/**
 * @brief XMacro to to loop over the fields of the telemetry packet
 * @param instance The name of the struct instance, used when listing the fields
 * @param X The macro to be called on each field
 */
#define TELEMETRY_PACKET(instance, X)         \
    X(instance, uint32_t, time_elapsed, , )   \
    X(instance, bool, ok, , )                 \
    X(instance, bool, sampling_state, , )     \
    X(instance, bool, temp_control_state, , ) \
    X(instance, float, target_temp, , )       \
    X(instance, float, current_temp, , )      \
    X(instance, float, current_pressure, , )  \
    X(instance, float, current_humidity, , )  \
    X(instance, float, battery_voltage, , )

#define CSV_HEADER_TELEMETRY "time,ok,sampling_state,temp_control_state,\
target_temp,current_temp,current_pressure,current_humidity,battery_voltage\n"
#define CSV_FORMAT_TELEMETRY "%u,%d,%d,%d,%f,%f,%f,%f,%f\n"

typedef struct __attribute__((packed))
{
    // Generate accelerometer struct
    ACCELEROMETER_PACKET(, FIELD, FIELD_ARRAY, )
} SD_packet_accelerometer;

typedef struct __attribute__((packed))
{
    // Generate telemetry struct
    TELEMETRY_PACKET(, FIELD)
} SD_packet_telemetry;

#endif // __PARSE_H
