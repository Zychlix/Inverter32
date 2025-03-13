#pragma once
#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"
#include "mitsubishi_dcdc_can.h"
#include "stm32f3xx.h"

typedef enum
{
    CHG_OK,
    CHG_FAIL
} chg_ret_val_t;


typedef enum
{
    CHG_DISABLED,
    CHG_UNINITIALIZED,
    CHG_IDLE,
} chg_state_t;


typedef enum
{
    CHG_ONLINE,
    CHG_OFFLINE,
} chg_power_state_t;

typedef struct
{
//    bool active; //Is dcdc controller running
    CAN_HandleTypeDef *can;
    chg_state_t state;

    DCDC_Converted_Data_t telemetry;
    DCDC_Charger_t frames;


} chg_t;

chg_ret_val_t static chg_switch_power(chg_t * instance, chg_power_state_t power); //Turn on or off the charger

chg_ret_val_t chg_initialize(chg_t * instance);

void chg_config_filters(chg_t *chg);

chg_ret_val_t chg_establish_connection(chg_t * instance); //The charger gets into idle state


void chg_message_semaphore(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[], chg_t * charger);

void chg_send_data(chg_t * charger);

chg_ret_val_t chg_refresh_data_struct(chg_t * instance);