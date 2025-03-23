#include "CAN_bus.h"

bool CAN_bus_init(struct CAN_bus_handler *c, uint32_t base_id)
{
    for (int i = 0; i < N_MESSAGES; i++)
    {
        c->Tx_headers[i].DLC = 8;
        c->Tx_headers[i].ExtId = 0;
        c->Tx_headers[i].IDE = CAN_ID_STD;
        c->Tx_headers[i].RTR = CAN_RTR_DATA;
        c->Tx_headers[i].StdId = base_id + i;
        c->Tx_headers[i].TransmitGlobalTime = DISABLE;
    }
    return true;
}

bool CAN_bus_receieve(struct CAN_bus_handler *c, CAN_HandleTypeDef *hcan1)
{
    HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &(c->Rx_header), c->Rx_data);
    c->command_ready = true;
    return true;
}

struct command CAN_bus_parse_command(struct CAN_bus_handler *c)
{
    struct command com;
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
    return com;
}