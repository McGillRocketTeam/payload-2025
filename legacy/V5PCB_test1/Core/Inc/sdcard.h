/*
 * sdcard.h
 *
 *  Created on: Dec 3, 2023
 *      Author: siger
 */

#ifndef INC_SDCARD_H_
#define INC_SDCARD_H_

#include <string.h>
#include <stdio.h>
#include <stdarg.h> //for va_list var arg functions


#define SERIAL_MONITOR 0

#define SD_FLUSH_LINE_SIZE 1
#define SD_FLUSH_TIME_FREQ_SEC 5

// Variables
extern FIL fil;  // File handle

extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart4;

// Function prototypes
void myprintf(const char *fmt, ...);
FRESULT SDCard_file_write(FIL *fil, uint8_t *readBuf, uint32_t size, uint16_t *line_written);
FRESULT SDCard_file_flush_line_check(FIL *fil, uint16_t *line_written);

#endif /* INC_SDCARD_H_ */
