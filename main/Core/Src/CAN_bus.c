#include "CAN_bus.h"
#include "enabled.h"
#include "serial_monitor.h"

// Temperature values are in degrees Celsius. To be finalized by: payload software + ground station teams
temperature temperatures[N_TEMPERATURES] = {1, 5, 10, 15, 20, 25, 30, 37};

bool CAN_bus_init(struct CAN_bus_handler *c, CAN_HandleTypeDef *hcan, uint32_t base_id)
{
#if CAN_BUS_ENABLED
	// Configure transmit headers for all three messages
	c->hcan = hcan;
    for (int i = 0; i < N_MESSAGES; i++)
    {
        c->Tx_headers[i].DLC = 8;
        c->Tx_headers[i].ExtId = 0;
        c->Tx_headers[i].IDE = CAN_ID_STD;
        c->Tx_headers[i].RTR = CAN_RTR_DATA;
        c->Tx_headers[i].StdId = base_id + i;
        c->Tx_headers[i].TransmitGlobalTime = DISABLE;
    }

    // Configure filter banks for receiving messages
    uint32_t command_ids[5] = {RESET_PAYLOAD, TOGGLE_SAMPLING, TOGGLE_COOLER, LANDED, SET_TEMPERATURE};
    CAN_FilterTypeDef filter_config;
    bool filter_config_success = true;
    for (int i = 0; i < 5; i++)
    {
		filter_config.FilterActivation = CAN_FILTER_ENABLE;
		filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
		filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
		filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
		filter_config.FilterBank = i;  // anything between 0 to SlaveStartFilterBank
		filter_config.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1
		filter_config.FilterIdHigh = command_ids[i]<<5;
		filter_config.FilterIdLow = 0x0000;
		filter_config.FilterMaskIdHigh = 0xFFFF<<5;
		filter_config.FilterMaskIdLow = 0x0000;
		filter_config_success &= HAL_CAN_ConfigFilter(hcan, &filter_config) == HAL_OK;
    }

    // Start peripheral and enable receipt callback
    HAL_StatusTypeDef start_status = HAL_CAN_Start(hcan);
    HAL_StatusTypeDef interrupt_status = HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    return start_status == HAL_OK && interrupt_status == HAL_OK && filter_config_success;
#else
    return true;
#endif
}

bool CAN_bus_receieve(struct CAN_bus_handler *c)
{
#if CAN_BUS_ENABLED
	bool received = HAL_CAN_GetRxMessage(c->hcan, CAN_RX_FIFO0, &(c->Rx_header), c->Rx_data) == HAL_OK;
    c->command_ready = received;
    return received;
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

    status1 = HAL_CAN_AddTxMessage(c->hcan, &((c->Tx_headers)[0]), msg1_u.bytes, &(c->Tx_mailbox));
    HAL_Delay(100);
    printf("err1: %ld, ", HAL_CAN_GetError(c->hcan));
    status2 = HAL_CAN_AddTxMessage(c->hcan, &((c->Tx_headers)[1]), msg2_u.bytes, &(c->Tx_mailbox));
    printf("err2: %ld, ", HAL_CAN_GetError(c->hcan));
    HAL_Delay(100);
    status3 = HAL_CAN_AddTxMessage(c->hcan, &((c->Tx_headers)[2]), msg3_u.bytes, &(c->Tx_mailbox));
    printf("err2: %ld\r\n", HAL_CAN_GetError(c->hcan));

    printf("status1: %d, status2: %d, status3: %d\r\n", status1, status2, status3);

    return status1 == HAL_OK && status2 == HAL_OK && status3 == HAL_OK;
#else
    return true;
#endif
}
