/**
 * @file CAN_bus.h
 * @author julien, Alex, Akash
 * @date 2025-03-15
 * @brief CAN bus message structures and unions. Unions are the same structures but available as an 64bit array.
 */
#ifndef __CAN_BUS_H
#define __CAN_BUS_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"

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
    RESET_PAYLOAD = 0x05, // Reset payload
    TOGGLE_SAMPLING = 0x11, // Toggle payload sampling
    TOGGLE_COOLER = 0x18, // Toggle peltier cooler
    LANDED = 0x17,
    SET_TEMPERATURE = 0x19, // Set target temperature. There are 8 possible temperature values. Encoded as 0-7.
    NONE = -1,
    INVALID = -2
};

#define N_TEMPERATURES 8
typedef float temperature;

union command_data
{
    bool on;
    temperature temp;
};

struct command
{
    enum command_type type;
    union command_data data;
};

struct CAN_bus_handler
{
	CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef Tx_headers[N_MESSAGES];
    CAN_RxHeaderTypeDef Rx_header;
    uint8_t Rx_data[8];
    bool command_ready;
};

bool CAN_bus_init(struct CAN_bus_handler *c, CAN_HandleTypeDef *hcan, uint32_t base_id);

bool CAN_bus_receive(struct CAN_bus_handler *c);

struct command CAN_bus_parse_command(struct CAN_bus_handler *c);

bool CAN_bus_send(
    struct CAN_bus_handler *c,
    bool ok,
    bool sampling_state,
    bool temperature_control_state,
    uint8_t target_temp,
    uint16_t current_temp,
    uint8_t battery_voltage,
    uint16_t frequency_x,
    uint16_t frequency_y,
    uint16_t frequency_z,
    uint16_t amplitude_x,
    uint16_t amplitude_y,
    uint16_t amplitude_z,
    uint32_t time_elapsed
);

#endif // __CAN_BUS_H
