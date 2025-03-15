/**
 * @file enabled.h
 * @author Akash Shah
 * @date 2025-03-15
 * @brief Globally enables or disables features. 
 *  If a feature is disabled, all functions will do nothing and return a success value if applicable.
 */

#ifndef __ENABLED_H
#define __ENABLED_H

// Comment out a line to not define and therefore disable a feature
#define CAN_BUS
#define SD_CARD
#define SERIAL_PRINT

#endif