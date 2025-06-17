/*
 * SD_card.c
 *
 *  Created on: Jun 16, 2025
 *      Author: akash
 */

#include <stdio.h>
#include "stm32f446xx.h"
#include "SD_card.h"
#include "enabled.h"

bool PL_SDCard_Init(PL_SDCard_Handler *sd_card, FATFS *fs, FIL *file)
{
#if SD_CARD_ENABLED
    // Ensure SD card is inserted (the GPIO pin is low when the card is inserted)
    if (HAL_GPIO_ReadPin(SD_DETECT_GPIO_Port, SD_DETECT_Pin))
    {
        // SD card is not inserted
        return false;
    }

    sd_card->fs = fs;
    sd_card->file = file;

    // Mount the filesystem
    // path = "" for the root directory
    // opt = 1 to force immediate mount
    return f_mount(sd_card->fs, "", 1) == FR_OK;
#else
    return true;
#endif
}

bool PL_SDCard_Open(PL_SDCard_Handler *sd_card)
{
#if SD_CARD_ENABLED
    // File name buffer needs one extra byte for the null terminator
    char name[SD_FILE_NAME_MAX_LENGTH + 1] = {0};

    // Check for the lowest available file name
    int i = 0;
    int len;
    FRESULT exists;
    FILINFO info;
    do
    {
        len = snprintf(name, sizeof(name), "%s%d.%s", SD_FILE_BASE_NAME, i, SD_FILE_EXTENSION);
        exists = f_stat(name, &info);
        i++;
    } while (exists == FR_OK);
    // Fail if the file name exceeds the maximum length
    if (len > SD_FILE_NAME_MAX_LENGTH)
    {
        return false;
    }

    // Create and open the file, return success status
    return f_open(sd_card->file, name, FA_CREATE_NEW | FA_WRITE) == FR_OK;
#else
    return true;
#endif
}

bool PL_SDCard_Close(PL_SDCard_Handler *sd_card)
{
#if SD_CARD_ENABLED
    return f_close(sd_card->file) == FR_OK;
#else
    return true;
#endif
}
