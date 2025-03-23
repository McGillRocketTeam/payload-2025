#include "CAN_bus.h"

bool CAN_bus_init(CAN_bus_handler *c, uint32_t base_id)
{
    for (int i = 0; i < N_MESSAGES; i++)
    {
        c->TxHeaders[i].DLC = 8;
        c->TxHeaders[i].ExtId = 0;
        c->TxHeaders[i].IDE = CAN_ID_STD;
        c->TxHeaders[i].RTR = CAN_RTR_DATA;
        c->TxHeaders[i].StdId = base_id + i;
        c->TxHeaders[i].TransmitGlobalTime = DISABLE;
    }
    return true;
}

bool CAN_bus_receieve(CAN_bus_handler *c, CAN_HandleTypeDef *hcan1)
{
    HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &(c->RxHeader), c->RxData);
    c->command_ready = true;
    return true;
}

command CAN_bus_parse_command(CAN_bus_handler *c)
{
    struct command com;
    union command_data data;
    if (c->command_ready)
    {
        switch (c->RxHeader.StdId)
        {
            case RESET:
                com.type = RESET;
            case TOGGLE_SAMPLING:
                com.type = TOGGLE_SAMPLING;
                data.on = c->RxData[0];
            case TOGGLE_COOLER:
                com.type = TOGGLE_COOLER;
                data.on = c->RxData[0];
            case LANDED:
                com.type = LANDED;
                data.on = c->RxData[0];
            case SET_TEMPERATURE:
                com.type = SET_TEMPERATURE;
                if (c->RxData[0]<N_TEMPERATURES){
                    data.temp = temperatures[c->RxData[0]];
                }
            default:
                com.type=INVALID;
        }
        com.data = data;
        return com;
    }
}