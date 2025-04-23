#include "CAN_bus.h"
#include "enabled.h"

bool CAN_bus_init(struct CAN_bus_handler *c, uint32_t base_id)
{
#if CAN_BUS_ENABLED
    for (int i = 0; i < N_MESSAGES; i++)
    {
        c->Tx_headers[i].DLC = 8;
        c->Tx_headers[i].ExtId = 0;
        c->Tx_headers[i].IDE = CAN_ID_STD;
        c->Tx_headers[i].RTR = CAN_RTR_DATA;
        c->Tx_headers[i].StdId = base_id + i;
        c->Tx_headers[i].TransmitGlobalTime = DISABLE;
    }
#endif
    return true;
}

bool CAN_bus_receieve(struct CAN_bus_handler *c, CAN_HandleTypeDef *hcan1)
{
#if CAN_BUS_ENABLED
    HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &(c->Rx_header), c->Rx_data);
    c->command_ready = true;
    return status == HAL_OK;
#else
    return true;
#endif
}

struct command CAN_bus_parse_command(struct CAN_bus_handler *c)
{
    struct command com;
    com.type = NONE;
#if CAN_BUS_ENABLED
    union command_data data;
    if (c->command_ready)
    {
        switch (c->Rx_header.StdId)
        {
        case RESET:
            com.type = RESET;
        case TOGGLE_SAMPLING:
            com.type = TOGGLE_SAMPLING;
            data.on = c->Rx_data[0];
        case TOGGLE_COOLER:
            com.type = TOGGLE_COOLER;
            data.on = c->Rx_data[0];
        case LANDED:
            com.type = LANDED;
        case SET_TEMPERATURE:
            com.type = SET_TEMPERATURE;
            if (c->Rx_data[0] < N_TEMPERATURES)
            {
                data.temp = temperatures[c->Rx_data[0]];
            }
        default:
            com.type = INVALID;
        }
    }
    com.data = data;
#endif
    return com;
}

bool CAN_bus_send(
    struct CAN_bus_handler *c,
    CAN_HandleTypeDef *hcan1,
    bool ok,
    bool sampling_state,
    bool temperature_control_state,
    uint8_t target_temp,
    uint16_t current_temp,
    uint8_t battery_voltage,
    uint16_t frquency_x,
    uint16_t frquency_y,
    uint16_t frquency_z,
    uint16_t amplitude_x,
    uint16_t amplitude_y,
    uint16_t amplitude_z,
    uint32_t time_elapsed)
{
#if CAN_BUS_ENABLED
    struct CAN_msg_1_s msg1;
    msg1.battery_voltage = battery_voltage;
    msg1.current_temp = current_temp;
    msg1.ok = ok;
    msg1.sampling_state = sampling_state;
    msg1.target_temp = target_temp;
    msg1.temp_ctrl_state = temperature_control_state;

    struct CAN_msg_2_s msg2;
    msg2.ampX = amplitude_x;
    msg2.frqX = frquency_x;
    msg2.frqY = frquency_y;
    msg2.frqZ = frquency_z;

    struct CAN_msg_3_s msg3;
    msg3.ampY = amplitude_y;
    msg3.ampZ = amplitude_z;
    msg3.time_elapsed = time_elapsed;

    union CAN_msg_1_u msg1_u;
    msg1_u.msg = msg1;

    union CAN_msg_2_u msg2_u;
    msg2_u.msg = msg2;

    union CAN_msg_3_u msg3_u;
    msg3_u.msg = msg3;

    HAL_StatusTypeDef status1;
    HAL_StatusTypeDef status2;
    HAL_StatusTypeDef status3;

    status1 = HAL_CAN_AddTxMessage(hcan1, &((c->Tx_headers)[0]), msg1_u.bytes, &(c->Tx_mailbox));
    status2 = HAL_CAN_AddTxMessage(hcan1, &((c->Tx_headers)[1]), msg2_u.bytes, &(c->Tx_mailbox));
    status3 = HAL_CAN_AddTxMessage(hcan1, &((c->Tx_headers)[2]), msg3_u.bytes, &(c->Tx_mailbox));

    return !(status1 != HAL_OK || status2 != HAL_OK || status3 != HAL_OK);
#else
    return true;
#endif
}
