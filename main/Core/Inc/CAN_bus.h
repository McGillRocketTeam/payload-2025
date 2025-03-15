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
#include "stm32f4xx_hal_can.h"

#define N_MESSAGES 3

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

enum command_type
{
    SCRUB,
    TOGGLE_SAMPLING,
    TOGGLE_COOLER,
    LANDED,
    SET_TEMP,
    // Add more commands as needed. There will be more commands in 2025
    INVALID = -1
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

union command_data {
    bool on;
    temperature temp;
};

struct command
{
    command_type type;
    command_data data;
};

struct CAN_bus_handler {
    CAN_TxHeaderTypeDef Tx_headers[N_MESSAGES];
    uint32_t Tx_mailbox;
    CAN_RxHeaderTypeDef Rx_header;
    uint8_t RxData[8];
    bool command_ready;
};

bool CAN_bus_init(CAN_bus_handler *c);

bool CAN_bus_receieve(CAN_bus_handler *c, CAN_HandleTypeDef *hcan1);

command CAN_bus_parse_command(CAN_bus_handler *c);

#endif // __CAN_BUS_H