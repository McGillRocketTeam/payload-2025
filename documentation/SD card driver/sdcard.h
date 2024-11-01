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

// Variables
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart2;
extern const UINT SD_FLUSH_LINE_SIZE;
extern const UINT SD_FLUSH_TIME_FREQ_SEC;

// TODO: modify if need to write to multiple files simultaneously
extern FIL fil;           // File handle
extern UINT line_written; // Counter for the lines written

// Function prototypes
void myprintf(const char *fmt, ...);
FRESULT SDCard_file_write(FIL *fil, BYTE *readBuf, UINT *line_written);
FRESULT SDCard_file_flush_line_check(FIL *fil, UINT *line_written);

#endif /* INC_SDCARD_H_ */
