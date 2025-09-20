#include "dcdc_controller.h"
#include "string.h"
//
// Created by zychlix on 11.03.25.
//


chg_ret_val_t chg_init(chg_t *instance) {
    if (!instance) {
        return CHG_FAIL;
    }

    if(instance->power.pin ==0 || instance->power.port == 0)
    {
        return CHG_FAIL;
    }

    return CHG_OK;
}

//void chg_message_semaphore(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[], chg_t *charger) {
//    uint32_t message_id = pHeader->StdId;
//    switch (message_id) {
//        case 0x377:
//            if (pHeader->DLC == sizeof(DCDC_Frame_Status_377_t)) {
//                memcpy(&charger->frames.status, aData, sizeof(DCDC_Frame_Status_377_t));
//                charger->frames.frame_377_received = true;
//            }
//            break;
//        case 0x389:
//            if (pHeader->DLC == sizeof(DCDC_Frame_Main_Battery_389_t)) {
//                memcpy(&charger->frames.main_battery_status, aData, sizeof(DCDC_Frame_Main_Battery_389_t));
//                charger->frames.frame_389_received = true;
//            }
//            break;
//        case 0x38A:
//            if (pHeader->DLC == sizeof(DCDC_Frame_Main_Battery_38A_t)) {
//                memcpy(&charger->frames.evse, aData, sizeof(DCDC_Frame_Main_Battery_38A_t));
//                charger->frames.frame_38A_received = true;
//            }
//
//        default:
//            break;
//
//    }
//}

void chg_message_semaphore(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[], chg_t *charger) {
    uint32_t message_id = pHeader->ExtId;

    charger->last_received_time = HAL_GetTick(); //Refresh last received timestamp

    switch (message_id) {
        case DEZHOU_STATUS_EXTID:
            if (pHeader->DLC == sizeof(dezhou_status_frame_t))
            {
                memcpy(&charger->status_frame, aData, sizeof(dezhou_status_frame_t));
                charger->status_frame_received=true;
            }
            break;

        default:
            break;

    }
}

// Check if it only catches Charger can
void chg_config_filters(chg_t *chg) {
    CAN_HandleTypeDef *hcan1 = chg->can;

    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //set fifo assignment
    sFilterConfig.FilterIdHigh = 0 << 5; //the ID that the filter looks for (switch this for the other microcontroller)
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = 0xffff;
    sFilterConfig.FilterMaskIdLow = 0xffff;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //set filter scale
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
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

void chg_deactivate_filters(chg_t *chg)
{
    CAN_HandleTypeDef * hcan1 = chg->can;

    CAN_FilterTypeDef sFilterConfig = {0};
    sFilterConfig.FilterActivation = ENABLE;

    sFilterConfig.FilterBank = 0;
    HAL_CAN_ConfigFilter(hcan1, &sFilterConfig);


    sFilterConfig.FilterBank = 1;
    HAL_CAN_ConfigFilter(hcan1, &sFilterConfig);


    sFilterConfig.FilterBank = 2;
    HAL_CAN_ConfigFilter(hcan1, &sFilterConfig);
}


uint16_t swap_endianness_16(uint16_t value) {
    return (value >> 8) | (value << 8);
}

void chg_send_slow_data(chg_t *charger) {
    CAN_HandleTypeDef *hcan1 = charger->can;
    uint32_t shit;

    DCDC_Frame_Enable_x285_t en_payload = {0};
    CAN_TxHeaderTypeDef tx_header = {0};


    DCDC_Frame_Setpoint_x286_t set_payload = {0};
    set_payload.charge_current = 2; //0x70 -> 120d max
//    set_payload.charge_voltage = swap_endianness_16(3500);
    set_payload.charge_voltage = 0x740e;
    set_payload._nn0[0] = 0x37; //magic number
    set_payload._nn0[1] = 0x00; //magic number
    set_payload._nn0[2] = 0x00; //magic number

    set_payload._nn0[3] = 0xa; //magic number
    set_payload._nn0[4] = 0x0; //magic number

    tx_header.StdId = 0x286;
    tx_header.DLC = sizeof(set_payload);

    HAL_CAN_AddTxMessage(hcan1, &tx_header, (uint8_t *) &set_payload, &shit);

    /*
     * What was the purpose of it?
     */
//    tx_header.StdId = 0x2FF;
//    uint8_t bytes[8];
//    bytes[0] = 0x01;
//    bytes[1] = 0xE8;
//    bytes[2] = 0x03;
//    bytes[3] = 0x50;
//    bytes[4] = 0x0F;
//    bytes[5] = 0x78; // 78=12A -> 0,1A/bit
//    bytes[6] = 0x00;
//    bytes[7] = 0x00;
//
//    HAL_CAN_AddTxMessage(hcan1, &tx_header, (uint8_t *) bytes, &shit);
}

uint32_t chg_send_data(chg_t *charger) {

    uint32_t shit;

    CAN_HandleTypeDef *hcan1 = charger->can;

    dezhou_command_frame_t en_payload = {0};
    CAN_TxHeaderTypeDef tx_header = {0};

    en_payload.voltage_setpoint_msb = swap_endianness_16((uint16_t)(charger->setpoint.voltage*10));
    en_payload.current_setpoint_msb = swap_endianness_16((uint16_t)(charger->setpoint.current*10));
    en_payload.mode = charger->setpoint.mode;
    en_payload.battery_contactor =  charger->setpoint.protection;

    tx_header.ExtId = DEZHOU_COMMAND_EXTID;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.DLC = sizeof(en_payload);
    tx_header.RTR = CAN_RTR_DATA;


    HAL_StatusTypeDef retval;
    retval = HAL_CAN_AddTxMessage(hcan1, &tx_header, (uint8_t *) &en_payload, &shit);

    if(retval == HAL_OK)
    {
        return 0;
    } else
    {
        return 1;
    }


}



chg_ret_val_t chg_refresh_data_struct(chg_t *instance) {
    if (!instance) {
        return CHG_FAIL;
    }

    if (instance->status_frame_received)
    {
        instance->status_frame_received = false;

        //tu parsuj

        DCDC_Converted_Data_t *data = &instance->telemetry;

        data->new_frame = true; //reset on read

        data->voltage = (float)swap_endianness_16(instance->status_frame.voltage_msb)/10.f;
        data->current = (float)swap_endianness_16(instance->status_frame.current_msb)/10.f;

        data->ac_error = instance->status_frame.ac_error;
        data->battery_error = instance->status_frame.battery_error;
        data->communication_timeout = instance->status_frame.communication_timeout;
        data->hardware_failure = instance->status_frame.hardware_failure;
        data->over_temperature = instance->status_frame.over_temperature;

    }

    return CHG_OK;
}


void chg_print_data(chg_t * instance)
{
    printf("\n\n\n Charger status: \n");

    printf("    CAN error: %d\n", instance->telemetry.communication_timeout);
    printf("    Battery voltage: %f\n", instance->telemetry.voltage);
    printf("    Battery current: %f\n", instance->telemetry.current);
    printf("    Temperature too high: %d\n", instance->telemetry.over_temperature);
    printf("    Battery error: %d\n", instance->telemetry.battery_error);
    printf("    Supply error: %d\n", instance->telemetry.ac_error);
    printf("    Last message: %d\n", instance->last_received_time);

}

chg_ret_val_t static chg_switch_power(chg_t * instance, bool power)
{
    if (!instance) {
        return CHG_FAIL;
    }

    HAL_GPIO_WritePin(instance->power.port, instance->power.pin, power);
    return CHG_OK;
}

chg_ret_val_t chg_state_machine_update(chg_t * instance)
{
    chg_refresh_data_struct(instance);
    uint32_t current_tick = HAL_GetTick();

    bool ready = (current_tick - instance->last_received_time)<CHG_TICK_MAX_TIMEOUT;

    switch (instance->current_command) {
        case CHG_CMD_ENABLE:
            if(instance->state == CHG_DISABLED)
            {
                chg_switch_power(instance, true);
            }
//            instance->state = CHG_UNINITIALIZED;
            break;

        case CHG_CMD_DISABLE:
            chg_switch_power(instance, false);
            instance->state = CHG_DISABLED;
            break;

        case CHG_CMD_START_CHARGING:

            if(instance->state == CHG_IDLE)
            {
                instance->state = CHG_WAITING_FOR_CHARGING;
            }

            break;

        default:
            break;


    }

    switch (instance->state) {
        case CHG_UNINITIALIZED:
            if(instance->telemetry.new_frame)
            {
                instance->state = CHG_IDLE;
            }
            break;

        case CHG_WAITING_FOR_CHARGING:

            if(instance->status_frame_received)
            {
                instance->state = CHG_CHARGING;
            }
            break;

        case CHG_CHARGING:


            chg_send_data(instance);
            break;
//            chg_send_slow_data(instance);
            //If AC_Present?
        default:
            break;

    }


//    if(charger.telemetry.ready_for_charging)
//    {
//        //chg_send_data(&charger);
//    }



    instance->current_command = CHG_CMD_NONE;
    return CHG_OK;

}

chg_ret_val_t chg_command(chg_t * instance, chg_command_t command)
{
    instance->current_command = command;
    return CHG_OK;
}

