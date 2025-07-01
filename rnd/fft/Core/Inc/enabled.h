/**
 * @file enabled.h
 * @author Akash Shah
 * @date 2025-03-15
 * @brief Globally enables or disables features. 
 *  If a feature is disabled, all functions will do nothing and return a success value if applicable.
 */

#ifndef __ENABLED_H
#define __ENABLED_H

// Set to 0 to disable a feature
#define CAN_BUS_ENABLED 1
#define SD_CARD_ENABLED 1
#define SERIAL_MONITOR_ENABLED 1
#define ADC_ENABLED 1
// Automatically disable accelerometer if ADC is disabled
#if ADC_ENABLED
#define ACCELEROMETER_ENABLED 1 // Change this value to toggle accelerometer 
#else
/*
 * Disable accelerometer if ADC is disabled
 * DO NOT CHANGE THIS!
*/
#define ACCELEROMETER_ENABLED 0 // 
#endif

#endif
