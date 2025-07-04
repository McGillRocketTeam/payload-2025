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

// Provide printf function prototype to prevent implicit definition warnings
int printf(const char *restrict format, ...);

extern UART_HandleTypeDef huart4;
#define SERIAL_MONITOR_UART huart4

// ANSI color escape codes
#define COLOR_RESET          "\x1b[0m"
#define COLOR_NONE           ""
#define COLOR_RED            "\x1b[31m"
#define COLOR_GREEN          "\x1b[32m"
#define COLOR_YELLOW         "\x1b[33m"
#define COLOR_BLUE           "\x1b[34m"
#define COLOR_MAGENTA        "\x1b[35m"
#define COLOR_CYAN           "\x1b[36m"
#define COLOR_BRIGHT_RED     "\x1b[91m"
#define COLOR_BRIGHT_GREEN   "\x1b[92m"
#define COLOR_BRIGHT_YELLOW  "\x1b[93m"
#define COLOR_BRIGHT_BLUE    "\x1b[94m"
#define COLOR_BRIGHT_MAGENTA "\x1b[95m"
#define COLOR_BRIGHT_CYAN    "\x1b[96m"

// Log type colors
#define COLOR_GENERAL            COLOR_NONE
#define COLOR_ACCELEROMETER      COLOR_YELLOW
#define COLOR_ADC                COLOR_BRIGHT_MAGENTA
#define COLOR_BLINK              COLOR_BRIGHT_CYAN
#define COLOR_CAN_BUS            COLOR_BLUE
#define COLOR_PELTIER            COLOR_BRIGHT_BLUE
#define COLOR_SD_CARD            COLOR_RED
#define COLOR_TEMPERATURE_SENSOR COLOR_MAGENTA
#define COLOR_DEBUG              COLOR_GREEN

enum log_type 
{
   LOG_GENERAL,
   LOG_ACCELEROMETER,
   LOG_ADC,
   LOG_BLINK,
   LOG_CAN_BUS,
   LOG_PELTIER,
   LOG_SD_CARD,
   LOG_TEMPERATURE_SENSOR,
   LOG_DEBUG = -1
};

// Log status colors
#define COLOR_INITIALIZING COLOR_MAGENTA
#define COLOR_OK           COLOR_GREEN
#define COLOR_WARNING      COLOR_YELLOW
#define COLOR_ERROR        COLOR_RED

enum log_status
{
   LOG_INITIALIZING,
   LOG_OK,
   LOG_WARNING,
   LOG_ERROR
};

int PL_Log(const char *restrict format, enum log_type type, enum log_status status, ...);

#endif /* INC_SERIAL_MONITOR_H_ */
