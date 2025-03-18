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
    set_payload.charge_current = 2;
    set_payload.charge_voltage = swap_endianness_16(3500);
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

        data->mains_present = instance->frames.main_battery_status.mains_present;
        data->charging = instance->frames.main_battery_status.charging;
        data->pilot_present = instance->frames.main_battery_status.pilot_present;
        data->can_error = instance->frames.main_battery_status.error_can;

        //aux battery
        data->aux_battery.charging_active = instance->frames.main_battery_status.dcdc_active;
        data->aux_battery.voltage = (float)swap_endianness_16(instance->frames.status.aux_battery_voltage)/100.f;
        data->aux_battery.current = (float)instance->frames.status.aux_current/10.f;

        data->main_battery_voltage = instance->frames.main_battery_status.battery_voltage;  //REAL VALUE
        data->main_battery_current = (float)instance->frames.main_battery_status.dc_current * 10;

        data->temperature.A0 =  (int16_t)(instance->frames.status.temperature_1 - 40);
        data->temperature.A1 =  (int16_t)(instance->frames.status.temperature_2 - 40);
        data->temperature.A2 =  (int16_t)(instance->frames.status.temperature_3 - 40);

        data->temperature.B0 =  instance->frames.main_battery_status.temperature_1;
        data->temperature.B0 =  instance->frames.main_battery_status.temperature_2;

        data->evse_duty = instance->frames.evse.evse_duty;

    }

    return CHG_OK;
}


void chg_print_data(chg_t * instance)
{
    printf("\n\n\n Charger status: \n");

    printf("\n\n Flags: \n");

    printf("    CAN error: %d\n", instance->telemetry.can_error);
    printf("    Ready for charging: %d\n", instance->telemetry.ready_for_charging);
    printf("    Waiting for mains: %d\n", instance->telemetry.waiting_for_mains);
    printf("    Pilot present: %d\n", instance->telemetry.pilot_present);
    printf("    Pilot duty: %d\n", instance->telemetry.evse_duty);
    printf("    Mains present: %d\n", instance->telemetry.mains_present);

    printf("\n\n Low voltage battery: \n");
    printf("    Voltage: %f\n", instance->telemetry.aux_battery.voltage);
    printf("    Current: %f\n", instance->telemetry.aux_battery.current);
    printf("    Charging: %d\n", instance->telemetry.aux_battery.charging_active);

    printf("\n\n Main battery: \n");
    printf("    Voltage: %f\n", (float)instance->telemetry.main_battery_voltage);
    printf("    Current: %f\n", instance->telemetry.main_battery_current);


    printf("\n\n Temperatures: \n");
    printf("    Temp A0: %d\n", instance->telemetry.temperature.A0);
    printf("    Temp A1: %d\n", instance->telemetry.temperature.A1);
    printf("    Temp A2: %d\n", instance->telemetry.temperature.A2);

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

    switch (instance->current_command) {
        case CHG_CMD_ENABLE:
            if(instance->state == CHG_DISABLED)
            {
                chg_switch_power(instance, true);
            }
            instance->state = CHG_UNINITIALIZED;
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
            chg_send_data(instance);
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
