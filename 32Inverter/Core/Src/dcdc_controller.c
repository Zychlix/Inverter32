#include "dcdc_controller.h"
#include "string.h"
//
// Created by zychlix on 11.03.25.
//


chg_ret_val_t chg_initialize(chg_t *instance) {
    if (!instance) {
        return CHG_FAIL;
    }

    return CHG_OK;
}

void chg_message_semaphore(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[], chg_t *charger) {
    uint32_t message_id = pHeader->StdId;
    switch (message_id) {
        case 0x377:
            if (pHeader->DLC == sizeof(DCDC_Frame_Status_377_t)) {
                memcpy(&charger->frames.status, aData, sizeof(DCDC_Frame_Status_377_t));
                charger->frames.frame_377_received = true;
            }
            break;
        case 0x389:
            if (pHeader->DLC == sizeof(DCDC_Frame_Main_Battery_389_t)) {
                memcpy(&charger->frames.main_battery_status, aData, sizeof(DCDC_Frame_Main_Battery_389_t));
                charger->frames.frame_389_received = true;
            }
            break;
        case 0x38A:
            if (pHeader->DLC == sizeof(DCDC_Frame_Main_Battery_38A_t)) {
                memcpy(&charger->frames.evse, aData, sizeof(DCDC_Frame_Main_Battery_38A_t));
                charger->frames.frame_38A_received = true;
            }

        default:
            break;

    }
}

void chg_config_filters(chg_t *chg) {
    CAN_HandleTypeDef *hcan1 = chg->can;

    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //set fifo assignment
    sFilterConfig.FilterIdHigh = 0x389 << 5; //the ID that the filter looks for (switch this for the other microcontroller)
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow = 0;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //set filter scale
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.FilterBank = 0;
    HAL_CAN_ConfigFilter(hcan1, &sFilterConfig); //configure CAN filter

    //filter2
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //set fifo assignment
    sFilterConfig.FilterIdHigh = 0x377 << 5; //the ID that the filter looks for (switch this for the other microcontroller)
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow = 0;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //set filter scale
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.FilterBank = 1;
    HAL_CAN_ConfigFilter(hcan1, &sFilterConfig); //configure CAN filter

    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //set fifo assignment
    sFilterConfig.FilterIdHigh = (0x38A) << 5; //the ID that the filter looks for (switch this for the other microcontroller)
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow = 0;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //set filter scale
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.FilterBank = 2;
    HAL_CAN_ConfigFilter(hcan1, &sFilterConfig); //configure CAN filter

    HAL_CAN_Start(hcan1);
    HAL_CAN_ActivateNotification(hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}


uint16_t swap_endianness_16(uint16_t value) {
    return (value >> 8) | (value << 8);
}

void chg_send_data(chg_t *charger) {
    CAN_HandleTypeDef *hcan1 = charger->can;
    uint32_t shit;

    DCDC_Frame_Enable_x285_t en_payload = {0};
    CAN_TxHeaderTypeDef tx_header = {0};
    tx_header.StdId = 0x285;
    tx_header.DLC = sizeof(en_payload);
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;

    en_payload.enable_0xb6 = 0xb6;

    HAL_CAN_AddTxMessage(hcan1, &tx_header, (uint8_t *) &en_payload, &shit);

    DCDC_Frame_Setpoint_x286_t set_payload = {0};
    set_payload.charge_current = 10;
    set_payload.charge_voltage = swap_endianness_16(3700);
    set_payload._nn0[0] = 0x37; //magic number
    set_payload._nn0[1] = 0x00; //magic number
    set_payload._nn0[2] = 0x00; //magic number

    set_payload._nn0[3] = 0xa; //magic number
    set_payload._nn0[4] = 0x0; //magic number

    tx_header.StdId = 0x286;
    tx_header.DLC = sizeof(set_payload);

    HAL_CAN_AddTxMessage(hcan1, &tx_header, (uint8_t *) &set_payload, &shit);

    tx_header.StdId = 0x2FF;
    uint8_t bytes[8];
    bytes[0] = 0x01;
    bytes[1] = 0xE8;
    bytes[2] = 0x03;
    bytes[3] = 0x50;
    bytes[4] = 0x0F;
    bytes[5] = 0x78; // 78=12A -> 0,1A/bit
    bytes[6] = 0x00;
    bytes[7] = 0x00;

    HAL_CAN_AddTxMessage(hcan1, &tx_header, (uint8_t *) bytes, &shit);
}


chg_ret_val_t chg_refresh_data_struct(chg_t *instance) {
    if (!instance) {
        return CHG_FAIL;
    }

    if (instance->frames.frame_377_received &&
        instance->frames.frame_38A_received &&
        instance->frames.frame_389_received) {
        instance->frames.frame_377_received = false;
        instance->frames.frame_38A_received = false;
        instance->frames.frame_389_received = false;

        //tu parsuj

        DCDC_Converted_Data_t *data = &instance->telemetry;

        data->new_frame = true; //reset on read
        data->in_operation = instance->frames.status.in_operation;
        data->ready = instance->frames.status.in_operation;

        data->mains_present = instance->frames.status.
//        bool can_error = data=;
        bool waiting_for_mains;
        bool ready_for_charging;
        bool pilot_present;

    }

    return CHG_OK;
}
