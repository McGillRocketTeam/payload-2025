/*
 * sdcard.c
 *
 *  Created on: Nov 26, 2023
 *      Author: siger
 */

#include "fatfs.h"
#include "sdcard.h"

/* Public variables */
FIL fil;
uint16_t line_written = 0;
volatile uint8_t flush_timer = 0;

/* Public Functions */

/**
 * Print to the serial monitor
 *
 * input: string and its arguments
 */
#if SERIAL_MONITOR
void myprintf(const char *fmt, ...)
{
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart4, (uint8_t *)buffer, len, -1);
}
#else
void myprintf(const char *fmt, ...)
{
	// Do nothing
}
#endif

/**
 * Write a single line to a file,  updating the line_written counter
 *
 * input:
 *   fil: opened file handle
 *   readBuf: line to be written
 *   line_written: counter for number of lines written
 * output:
 *   fres: see FRESULT from ff.h for definition
 */
FRESULT SDCard_file_write(FIL *fil, uint8_t *readBuf, uint32_t size, uint16_t *line_written) {
	FRESULT fres;
	UINT bytesWrote;
	fres = f_write(fil, readBuf, size, &bytesWrote);

	if (fres == FR_OK)
	{
		(*line_written)++;
		myprintf("Wrote %i bytes, %u lines written since last flush\r\n", bytesWrote, *line_written);
	}
	else
	{
		myprintf("f_write error (%i)\r\n", fres);
	}

	return fres;
}

/**
 * Flush file when number of lines written reaches SD_FLUSH_LINE_SIZE
 *
 * input:
 *   fil: opened file handle
 *   line_written: counter for number of lines written
 * output:
 *   fres: see FRESULT from ff.h for definition
 */
FRESULT SDCard_file_flush_line_check(FIL *fil, uint16_t *line_written) {
	FRESULT fres = FR_OK;

	if (*line_written >= SD_FLUSH_LINE_SIZE)
	{
		// Flush
		fres = f_sync(fil);

		if (fres == FR_OK)
		{
			myprintf("Flushed after %i lines written\r\n", *line_written);
			(*line_written) = 0;
			flush_timer = 0;
		}
		else
		{
			myprintf("f_sync error (%i)\r\n", fres);
		}
	}

	return fres;
}


