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
    CHG_WAITING_FOR_CHARGING
} chg_state_t;


typedef enum
{
    CHG_ONLINE,
    CHG_OFFLINE,
} chg_power_state_t;

typedef struct
{
    uint32_t pin;
    GPIO_TypeDef * port;

}dcdc_pin_t;


typedef enum
{
    CHG_CMD_NONE,
    CHG_CMD_DISABLE,
    CHG_CMD_ENABLE,
    CHG_CMD_START_CHARGING,

} chg_command_t;

typedef struct
{
//    bool active; //Is dcdc controller running
    CAN_HandleTypeDef *can;
    dcdc_pin_t power;
    chg_command_t current_command;

    chg_state_t state;
    DCDC_Converted_Data_t telemetry;
    DCDC_Charger_t frames;

    bool slow_data_enabled;
    bool fast_data_enabled;

} chg_t;

chg_ret_val_t static chg_switch_power(chg_t * instance, bool power); //Turn on or off the charger

chg_ret_val_t chg_init(chg_t * instance);

void chg_config_filters(chg_t *chg);

chg_ret_val_t chg_establish_connection(chg_t * instance); //The charger gets into idle state


void chg_message_semaphore(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[], chg_t * charger);

void chg_send_slow_data(chg_t * charger);
void chg_send_fast_data(chg_t * charger);

chg_ret_val_t chg_refresh_data_struct(chg_t * instance);

chg_ret_val_t chg_state_machine_update(chg_t * instance);

chg_ret_val_t chg_command(chg_t * instance, chg_command_t command);

void chg_print_data(chg_t * instance);

chg_ret_val_t chg_start_pwm(chg_t * instance);
