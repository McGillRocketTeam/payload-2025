/**
 * @file CAN_bus.h
 * @author julien, Alex
 * @date 2025-03-15
 * @brief CAN bus message structures and unions. Unions are the same structures but available as an 64bit array.
 */
#ifndef __CAN_BUS_H
#define __CAN_BUS_H
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

// CAN Message structures
struct CAN_msg_1_s
{
    bool ok;
    bool sampling_state;
    bool temp_ctrl_state;
    uint8_t target_temp;
    uint16_t current_temp;
    uint8_t battery_voltage; // Data type not specified on interfacing requirements, range of 10.0-25.0V required
};

struct CAN_msg_2_s
{
    uint16_t frqX;
    uint16_t frqY;
    uint16_t frqZ;
    uint16_t ampX;
};

struct CAN_msg_3_s
{
    uint16_t ampY;
    uint16_t ampZ;
    uint32_t time_elapsed;
};

// Message unions
union CAN_msg_1_u
{
    struct CAN_msg_1_s msg;
    uint8_t bytes[8];
};

union CAN_msg_2_u
{
    struct CAN_msg_2_s msg;
    uint8_t bytes[8];
};

union CAN_msg_3_u
{
    struct CAN_msg_3_s msg;
    uint8_t bytes[8];
};

enum command
{
    SCRUB = 0x05, 
    TOGGLE_SAMPLING = 0x11,
    TOGGLE_COOLER, //TO BE ADDED
    LANDED = 0x17,
    INVALID = -1
    // Add more commands as needed. There will be more commands in 2025
};

// Temperature values are in degrees Celsius. To be finalized by: payload software + ground station teams
enum temperature
{
    TEMP_1 = 1,
    TEMP_2 = 5,
    TEMP_3 = 10,
    TEMP_4 = 15,
    TEMP_5 = 20,
    TEMP_6 = 25,
    TEMP_7 = 30,
    TEMP_8 = 37
};

#endif // __CAN_BUS_H