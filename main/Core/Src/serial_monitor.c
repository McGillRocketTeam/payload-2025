/*
 * serial_monitor.c
 *
 *  Created on: Apr 21, 2025
 *      Author: akash
 */

#include <stdarg.h>
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

int PL_Log(enum log_type type, enum log_status status, const char *restrict format, ...)
{
#if SERIAL_MONITOR_ENABLED
  // Print logging time left padded with 8 character limit
  if (printf("[%8ld] ", HAL_GetTick()) == EOF)
  {
    // Fail early if printf failed
    return EOF;
  }

  char *color;

  // Print logging type
  char *type_string;
  switch (type)
  {
  case LOG_GENERAL:
    type_string = "GENERAL";
    color = COLOR_GENERAL; 
    break;
  case LOG_ACCELEROMETER:
    type_string = "ACCELEROMETER";
    color = COLOR_ACCELEROMETER;
    break;
  case LOG_ADC:
    type_string = "ADC";
    color = COLOR_ADC;
    break;
  case LOG_BLINK:
    type_string = "BLINK";
    color = COLOR_BLINK;
    break;
  case LOG_CAN_BUS:
    type_string = "CAN BUS";
    color = COLOR_CAN_BUS;
    break;
  case LOG_PELTIER:
    type_string = "PELTIER";
    color = COLOR_PELTIER;
    break;
  case LOG_SD_CARD:
    type_string = "SD CARD";
    color = COLOR_SD_CARD;
    break;
  case LOG_TEMPERATURE_SENSOR:
    type_string = "TEMP SENSOR";
    color = COLOR_TEMPERATURE_SENSOR;
    break;
  case LOG_DEBUG:
    type_string = "DEBUG";
    color = COLOR_DEBUG;
    break;
  }
  // Print log type right padded by length of longest string
  if (printf("[%s%-13s%s] ", color, type_string, COLOR_RESET) == EOF)
  {
    // Fail early if printf failed
    return EOF;
  }

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
  if (printf("[%s%-5s%s] ", color, status_string, COLOR_RESET) == EOF)
  {
    // Fail early if printf failed
    return EOF;
  }

  // Initialize variable arg list
  va_list args;
  // `va_start` requires the last non-variadic argument
  va_start(args, format);
  
  // Write string formatted by va list into the string buffer
  static char buffer[256];
  vsnprintf(buffer, sizeof(buffer), format, args);

  // Release `va_list`
  va_end(args);

  // Print formatted message and newline, return success
  return printf("%s\r\n", buffer);
#endif
}
