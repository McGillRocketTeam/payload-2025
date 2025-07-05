/*
 * serial_monitor.c
 *
 *  Created on: Apr 21, 2025
 *      Author: akash
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "serial_monitor.h"
#include "enabled.h"

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
#if SERIAL_MONITOR_ENABLED
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART1 and Loop until the end of transmission */
    HAL_UART_Transmit(&SERIAL_MONITOR_UART, (uint8_t *)&ch, 1, 0xFFFF);
#endif
    return ch;
}

/*
 * Comment out a category to filter it out. 
 * If a category is automatically disabled because the feature is, but you don't want it to be filtered out,
 * add it to the end.
 */ 
const enum log_category category_filter = 
    LOG_GENERAL 
// Automatically filter out disabled features.
#if ACCELEROMETER_ENABLED
    | LOG_ACCELEROMETER
#endif
#if ADC_ENABLED
    | LOG_ADC
#endif
#if BLINK_ENABLED
    | LOG_BLINK
#endif
#if CAN_BUS_ENABLED
    | LOG_CAN_BUS
#endif
#if PELTIER_ENABLED
    | LOG_PELTIER
#endif
#if SD_CARD_ENABLED
    | LOG_SD_CARD
#endif
#if TEMPERATURE_SENSOR_ENABLED
    | LOG_TEMPERATURE_SENSOR
#endif
    | LOG_DEBUG;

// Remove a status from here to filter it out when logging
const enum log_status status_filter = LOG_INITIALIZING | LOG_OK | LOG_WARNING | LOG_ERROR;

int PL_Log(enum log_category category_primary,
           enum log_category other_categories,
           enum log_status status,
           const char *restrict format,
           ...)
{
#if SERIAL_MONITOR_ENABLED
    // Determine whether this message should be logged or ignored
    bool enabled;
    // Combine all categories into one
    enum log_category categories = category_primary | other_categories;
#if FILTER_MODE == FILTER_MODE_LENIENT
    // Checks if any bits (categories) are still enabled after being put through the filter
    // True if any categories of this message are enabled
    enabled = categories & category_filter;
#elif FILTER_MODE == FILTER_MODE_STRICT
    // Checks if the bits (categories) are unchanged after being put through the filter
    // True if all categories of this message are enabled
    enabled = (categories & category_filter) == categories;
#elif FILTER_MODE == FILTER_MODE_PRIMARY
    // Checks if the primary category's bit is enabled in the filter
    // True if the primary category is enabled
    enabled = category_primary & category_filter;
#endif

    // Determine whether the status is enabled or disabled
    enabled = enabled && (status & status_filter);

    // Ignore this message if enabled
    if (!enabled)
    {
        return 0;
    }

    // Track number of characters written
    int chars_written = 0;
    int new_chars;

    // Print logging time left padded with 8 character limit
    new_chars = printf("%s[%s%8ld%s]%s ",
                       COLOR_WHITE,
                       COLOR_RESET,
                       HAL_GetTick(),
                       COLOR_WHITE,
                       COLOR_RESET);
    if (new_chars == EOF)
    {
        // Fail early if printf failed
        return EOF;
    }
    chars_written += new_chars;

    // Next color to log in
    char *color;

    // Print logging category
    char *category_string;
    switch (category_primary)
    {
    case LOG_NONE:
    case LOG_GENERAL:
        category_string = "GENERAL";
        color = COLOR_GENERAL;
        break;
    case LOG_ACCELEROMETER:
        category_string = "ACCELEROMETER";
        color = COLOR_ACCELEROMETER;
        break;
    case LOG_ADC:
        category_string = "ADC";
        color = COLOR_ADC;
        break;
    case LOG_BLINK:
        category_string = "BLINK";
        color = COLOR_BLINK;
        break;
    case LOG_CAN_BUS:
        category_string = "CAN BUS";
        color = COLOR_CAN_BUS;
        break;
    case LOG_PELTIER:
        category_string = "PELTIER";
        color = COLOR_PELTIER;
        break;
    case LOG_SD_CARD:
        category_string = "SD CARD";
        color = COLOR_SD_CARD;
        break;
    case LOG_TEMPERATURE_SENSOR:
        category_string = "TEMP SENSOR";
        color = COLOR_TEMPERATURE_SENSOR;
        break;
    case LOG_DEBUG:
        category_string = "DEBUG";
        color = COLOR_DEBUG;
        break;
    }
    // Print log type right padded by length of longest string
    new_chars = printf("%s[%s%s%-13s%s%s]%s ",
                       COLOR_WHITE,
                       COLOR_RESET,
                       color,
                       category_string,
                       COLOR_RESET,
                       COLOR_WHITE,
                       COLOR_RESET);
    if (new_chars == EOF)
    {
        // Fail early if printf failed
        return EOF;
    }
    chars_written += new_chars;

    // Print logging status
    char *status_string;
    switch (status)
    {
    case LOG_INITIALIZING:
        status_string = "INIT";
        color = COLOR_INITIALIZING;
        break;
    case LOG_OK:
        status_string = "OK";
        color = COLOR_OK;
        break;
    case LOG_WARNING:
        status_string = "WARN";
        color = COLOR_WARNING;
        break;
    case LOG_ERROR:
        status_string = "ERROR";
        color = COLOR_ERROR;
        break;
    }
    // Print log status right padded by length of longest string
    new_chars = printf("%s[%s%s%-5s%s%s]%s ",
                       COLOR_WHITE,
                       COLOR_RESET,
                       color,
                       status_string,
                       COLOR_RESET,
                       COLOR_WHITE,
                       COLOR_RESET);
    if (new_chars == EOF)
    {
        // Fail early if printf failed
        return EOF;
    }
    chars_written += new_chars;

    // Initialize variable arg list
    va_list args;
    // `va_start` requires the last non-variadic argument
    va_start(args, format);

    // Write string formatted by va list into the string buffer
    static char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);

    // Release `va_list`
    va_end(args);

    // Print formatted message
    new_chars = printf("%s\r\n", buffer);
    if (new_chars == EOF)
    {
        return EOF;
    }
    else
    {
        return chars_written + new_chars;
    }
#else
    return 0;
#endif
}
