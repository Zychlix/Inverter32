#pragma once
#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"
#include "mitsubishi_dcdc_can.h"
#include "dezhou_charger_can.h"
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
    CHG_WAITING_FOR_CHARGING,
    CHG_CHARGING
} chg_state_t;

typedef enum
{
    DEZHOU_BATTERY_OPEN_CHARGING = 0,
    DEZHOU_BATTERY_PROTECTION = 1,

} dezhou_battery_protection_t;

typedef enum
{
    DEZHOU_MODE_CHARGING = 0,
    DEZHOU_MODE_HEATING = 1,

} dezhou_mode_protection_t;

typedef struct
{
    float voltage;
    float current;
    dezhou_battery_protection_t protection;
    dezhou_mode_protection_t mode;

}chg_setpoint_t;

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

typedef struct CHG
{
//    bool active; //Is dcdc controller running
    CAN_HandleTypeDef *can;
    dcdc_pin_t power;
    chg_command_t current_command;

    chg_state_t state;
    DCDC_Converted_Data_t telemetry;
//    DCDC_Charger_t frames;

    dezhou_status_frame_t status_frame;
    bool status_frame_received;

    chg_setpoint_t setpoint;

//    bool slow_data_enabled;
//    bool fast_data_enabled;

} chg_t;

chg_ret_val_t static chg_switch_power(chg_t * instance, bool power); //Turn on or off the charger

chg_ret_val_t chg_init(chg_t * instance);

void chg_config_filters(chg_t *chg);

void chg_deactivate_filters(chg_t * chg);


chg_ret_val_t chg_establish_connection(chg_t * instance); //The charger gets into idle state


void chg_message_semaphore(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[], chg_t * charger);

void chg_send_slow_data(chg_t * charger);

uint32_t chg_send_data(chg_t * charger);

chg_ret_val_t chg_refresh_data_struct(chg_t * instance);

chg_ret_val_t chg_state_machine_update(chg_t * instance);

chg_ret_val_t chg_command(chg_t * instance, chg_command_t command);

void chg_print_data(chg_t * instance);

chg_ret_val_t chg_start_pwm(chg_t * instance);
