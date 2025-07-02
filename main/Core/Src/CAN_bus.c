#include "CAN_bus.h"
#include "enabled.h"

bool PL_CANBus_Init(PL_CANBus_Handler *c, CAN_HandleTypeDef *hcan)
{
#if CAN_BUS_ENABLED
    // Initialize CAN bus handler
    c->hcan = hcan;
    c->command_ready = false;

    // Configure transmit headers for all three messages
    for (int i = 0; i < N_MESSAGES; i++)
    {
        c->Tx_headers[i].DLC = 8;
        c->Tx_headers[i].ExtId = 0;
        c->Tx_headers[i].IDE = CAN_ID_STD;
        c->Tx_headers[i].RTR = CAN_RTR_DATA;
        c->Tx_headers[i].StdId = CAN_BASE_ID + i;
        c->Tx_headers[i].TransmitGlobalTime = DISABLE;
    }

    // Configure filter banks for receiving messages
    uint32_t command_ids[6] = {RESET_PAYLOAD, TOGGLE_SAMPLING, TOGGLE_COOLER, TOGGLE_LAUNCH_MODE, LANDED, SET_TEMPERATURE};
    CAN_FilterTypeDef filter_config;
    bool filter_config_success = true;
    for (int i = 0; i < 6; i++)
    {
        filter_config.FilterActivation = CAN_FILTER_ENABLE;
        filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
        filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
        filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
        filter_config.FilterBank = i;            // anything between 0 to SlaveStartFilterBank
        filter_config.SlaveStartFilterBank = 13; // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1
        filter_config.FilterIdHigh = command_ids[i] << 5;
        filter_config.FilterIdLow = 0x0000;
        filter_config.FilterMaskIdHigh = 0xFFFF << 5;
        filter_config.FilterMaskIdLow = 0x0000;

        filter_config_success &= HAL_CAN_ConfigFilter(hcan, &filter_config) == HAL_OK;
    }

    // Start peripheral and enable message receipt callback
    HAL_StatusTypeDef start_status = HAL_CAN_Start(hcan);
    HAL_StatusTypeDef interrupt_status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    return start_status == HAL_OK && interrupt_status == HAL_OK && filter_config_success;
#else
    return true;
#endif
}

bool PL_CANBus_Receive(PL_CANBus_Handler *c)
{
#if CAN_BUS_ENABLED
    bool received = HAL_CAN_GetRxMessage(c->hcan, CAN_RX_FIFO0, (CAN_RxHeaderTypeDef *) &(c->Rx_header), (uint8_t *) c->Rx_data) == HAL_OK;
    c->command_ready = received;
    return received;
#else
    return true;
#endif
}

struct command PL_CANBus_ParseCommand(PL_CANBus_Handler *c)
{
    struct command com;
    com.type = NONE;

#if CAN_BUS_ENABLED
    union command_data data;
    if (c->command_ready)
    {
        switch (c->Rx_header.StdId)
        {
        case RESET_PAYLOAD:
            com.type = RESET_PAYLOAD;
            break;
        case TOGGLE_SAMPLING:
            com.type = TOGGLE_SAMPLING;
            data.on = c->Rx_data[0];
            break;
        case TOGGLE_COOLER:
            com.type = TOGGLE_COOLER;
            data.on = c->Rx_data[0];
            break;
        case TOGGLE_LAUNCH_MODE:
            com.type = TOGGLE_LAUNCH_MODE;
            data.on = c->Rx_data[0];
            break;
        case LANDED:
            com.type = LANDED;
            break;
        case SET_TEMPERATURE:
            com.type = SET_TEMPERATURE;
            if (c->Rx_data[0] < N_TEMPERATURES)
            {
                data.temp = temperatures[c->Rx_data[0]];
            }
            break;
        default:
            com.type = INVALID;
        }
    }
    com.data = data;
    c->command_ready = false;
#endif
    return com;
}

bool PL_CANBus_Send(
    PL_CANBus_Handler *c,
    bool ok,
    bool sampling_state,
    bool temperature_control_state,
    uint8_t target_temp,
    int16_t current_temp,
    uint8_t battery_voltage,
    uint16_t frequency_x,
    uint16_t frequency_y,
    uint16_t frequency_z,
    uint16_t amplitude_x,
    uint16_t amplitude_y,
    uint16_t amplitude_z,
    uint32_t time_elapsed
)
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
    msg2.frqX = frequency_x;
    msg2.frqY = frequency_y;
    msg2.frqZ = frequency_z;

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

    uint32_t Tx_mailbox;

    status1 = HAL_CAN_AddTxMessage(c->hcan, &((c->Tx_headers)[0]), msg1_u.bytes, &Tx_mailbox);
    uint32_t time_message_sent = HAL_GetTick();
    uint32_t current;
    do
    {
        current = HAL_GetTick();
    } while (HAL_CAN_IsTxMessagePending(c->hcan, Tx_mailbox) && current - time_message_sent < CAN_SEND_TIMEOUT);

    status2 = HAL_CAN_AddTxMessage(c->hcan, &((c->Tx_headers)[1]), msg2_u.bytes, &Tx_mailbox);
    time_message_sent = HAL_GetTick();
    do
    {
        current = HAL_GetTick();
    } while (HAL_CAN_IsTxMessagePending(c->hcan, Tx_mailbox) && current - time_message_sent < CAN_SEND_TIMEOUT);

    status3 = HAL_CAN_AddTxMessage(c->hcan, &((c->Tx_headers)[2]), msg3_u.bytes, &Tx_mailbox);
    time_message_sent = HAL_GetTick();
    do
    {
        current = HAL_GetTick();
    } while (HAL_CAN_IsTxMessagePending(c->hcan, Tx_mailbox) && current - time_message_sent < CAN_SEND_TIMEOUT);

    return status1 == HAL_OK && status2 == HAL_OK && status3 == HAL_OK;
#else
    return true;
#endif
}
