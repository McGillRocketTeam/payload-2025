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

#endif /* INC_SERIAL_MONITOR_H_ */
