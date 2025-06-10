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
#define CAN_SEND_TIMEOUT 100

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
	TOGGLE_LAUNCH_MODE = 0x33,
    LANDED = 0x17,
    SET_TEMPERATURE = 0x19, // Set target temperature. There are 8 possible temperature values. Encoded as 0-7.
    NONE = -1,
    INVALID = -2
};

#define N_TEMPERATURES 8

union command_data
{
    bool on;
    float temp;
};

struct command
{
    enum command_type type;
    union command_data data;
};

typedef struct
{
	CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef Tx_headers[N_MESSAGES];
    CAN_RxHeaderTypeDef Rx_header;
    uint8_t Rx_data[8];
    bool command_ready;
} PL_CANBus_Handler;

/**
 * @brief Initializes the CAN bus with the given CAN peripheral and base ID.
 * @param c Pointer to the CAN bus handler structure.
 * @param hcan Pointer to the CAN peripheral handle.
 * @param base_id Base ID for the CAN messages. Message ID is the base ID for the first message, and subsequent messages are assigned sequential IDs.
 * @return true if initialization is successful, false otherwise.
 */
bool PL_CANBus_Init(PL_CANBus_Handler *c, CAN_HandleTypeDef *hcan, uint32_t base_id);

/**
 * @brief Receives a CAN message and updates the command_ready flag.
 * @param c Pointer to the CAN bus handler structure.
 * @note This function should be called in the CAN receive interrupt callback.
 * @return true if a message was received, false otherwise.
 */
bool PL_CANBus_Receive(PL_CANBus_Handler *c);

/**
 * @brief Parses the received CAN message and returns the command.
 * @param c Pointer to the CAN bus handler structure.
 * @return A command structure containing the command type and data.
 * @note The command_ready flag must be set to true before calling this function.
 */
struct command PL_CANBus_ParseCommand(PL_CANBus_Handler *c);

/**
 * @brief Sends a CAN message with the specified parameters.
 * @param c Pointer to the CAN bus handler structure.
 * @param ok Indicates if the payload is functioning correctly.
 * @param sampling_state Indicates if the payload is currently sampling.
 * @param temperature_control_state Indicates if the temperature control is active.
 * @param target_temp The target temperature to be set (0-7).
 * @param current_temp The current temperature of the payload.
 * @param battery_voltage The current battery voltage.
 * @param frequency_x The frequency of the X-axis vibrations.
 * @param frequency_y The frequency of the Y-axis vibrations.
 * @param frequency_z The frequency of the Z-axis vibrations.
 * @param amplitude_x The amplitude of the X-axis vibrations.
 * @param amplitude_y The amplitude of the Y-axis vibrations.
 * @param amplitude_z The amplitude of the Z-axis vibrations.
 * @param time_elapsed The current time elapsed since the payload started.
 * @return true if the message was sent successfully, false otherwise.
 * @note This function sends three separate CAN messages with the provided parameters.
 * @note The function will block until the messages are sent or the timeout is reached.
 * @note This function should be called periodically or in a timer interrupt.
 */
bool PL_CANBus_Send(
    PL_CANBus_Handler *c,
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
