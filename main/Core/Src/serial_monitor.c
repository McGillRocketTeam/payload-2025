/*
 * serial_monitor.c
 *
 *  Created on: Apr 21, 2025
 *      Author: akash
 */

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
