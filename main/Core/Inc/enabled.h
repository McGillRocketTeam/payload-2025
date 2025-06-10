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
#define FINAL_BUILD 0

// Independent features
#define BLINK_ENABLED 1
#define CAN_BUS_ENABLED 1
#define PELTIER_ENABLED 1
#define SD_CARD_ENABLED 1
#define TEMPERATURE_SENSOR_ENABLED 0

// SERIAL_MONITOR_ENABLED
#if FINAL_BUILD
/*
 * Automatically disable serial monitor in final build.
 * DO NOT CHANGE THIS VALUE.
*/
#define SERIAL_MONITOR_ENABLED 0
#else
// Change this value
#define SERIAL_MONITOR_ENABLED 1
#endif

#endif
