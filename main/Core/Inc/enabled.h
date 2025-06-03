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
#define PELTIER_ENABLED 1
#define SERIAL_MONITOR_ENABLED 1
#define TEMPERATURE_SENSOR_ENABLED 1
#endif
