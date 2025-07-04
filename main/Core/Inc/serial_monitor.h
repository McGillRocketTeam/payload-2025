/*
 * serial_monitor.h
 *
 *  Created on: Apr 21, 2025
 *      Author: akash
 */

#ifndef INC_SERIAL_MONITOR_H_
#define INC_SERIAL_MONITOR_H_

#include "stm32f4xx.h"

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

extern UART_HandleTypeDef huart4;
#define SERIAL_MONITOR_UART huart4

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

enum log_category
{
   LOG_GENERAL            = 0b000000001u,
   LOG_ACCELEROMETER      = 0b000000010u,
   LOG_ADC                = 0b000000100u,
   LOG_BLINK              = 0b000001000u,
   LOG_CAN_BUS            = 0b000010000u,
   LOG_PELTIER            = 0b000100000u,
   LOG_SD_CARD            = 0b001000000u,
   LOG_TEMPERATURE_SENSOR = 0b010000000u,
   LOG_DEBUG              = 0b100000000u,
   LOG_NONE               = 0b000000000u
};

#define N_CATEGORIES 9
// To change filters, see `serial_monitor.c`.
extern const enum log_category category_filter;

// Log category colors
#define COLOR_GENERAL            COLOR_NONE
#define COLOR_ACCELEROMETER      COLOR_YELLOW
#define COLOR_ADC                COLOR_GREEN
#define COLOR_BLINK              COLOR_BRIGHT_CYAN
#define COLOR_CAN_BUS            COLOR_CYAN
#define COLOR_PELTIER            COLOR_BRIGHT_BLUE
#define COLOR_SD_CARD            COLOR_RED
#define COLOR_TEMPERATURE_SENSOR COLOR_MAGENTA
#define COLOR_DEBUG              COLOR_BRIGHT_GREEN

enum log_status
{
   LOG_INITIALIZING = 0b0001u,
   LOG_OK           = 0b0010u,
   LOG_WARNING      = 0b0100u,
   LOG_ERROR        = 0b1000u
};

// To change filters, see `serial_monitor.c`.
extern const enum log_status status_filter;

// Log status colors
#define COLOR_INITIALIZING COLOR_MAGENTA
#define COLOR_OK           COLOR_BRIGHT_GREEN
#define COLOR_WARNING      COLOR_BRIGHT_YELLOW
#define COLOR_ERROR        COLOR_BRIGHT_RED

// Provide printf function prototype to prevent implicit definition warnings
int printf(const char *restrict format, ...);

/**
 * @brief Logs the current time, primary category, status, and the formatted string to the serial monitor.
 * Filters messages based on the bitwise filters found in `serial_monitor.c`.
 * If any category or status of a message has its bit of the filter set to `0`, the message will be ignored.
 * A category is automatically filtered out if the corresponding feature is disabled in `enabled.h`.
 * @param category_primary The primary category of the message to log. This will be the one which is printed.
 * @param other_categories Set of other categories to which this message belongs. Used for message filtering.
 * Can be passed as a single `log_category` or as a bitwise or of multiple of them. 
 * If no other category applies, use `LOG_NONE` or `0`.
 * @param status The status applying to the message being logged. 
 * Is displayed in the message as well as being used for message filtering.
 * @param format The format string, as used in `printf`.
 * @param ... The values used to format the string.
 * @return Number of characters written if successful, `EOF` otherwise.
 * @example ```c
 * PL_Log(LOG_CAN_BUS, LOG_GENERAL | LOG_ACCELEROMETER, LOG_OK, "Hello %s %d!", "world", 3);
 * PL_Log(LOG_GENERAL, LOG_NONE, LOG_INITIALIZING, "Initializing...");
 * ```
 */
int PL_Log(enum log_category category_primary,
           enum log_category other_categories,
           enum log_status status,
           const char *restrict format,
           ...);

#endif /* INC_SERIAL_MONITOR_H_ */
