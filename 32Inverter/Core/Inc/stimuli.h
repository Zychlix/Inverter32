#pragma once
#include "stdbool.h"
#include "adc.h"

//This library contains event handlers

#define INV_MAX_TEMPERATURE_DISABLE 35.f //C
#define INV_MAX_TEMPERATURE_ENABLE 45.f //C

#define ENV_MIN_VBUS_VALUE 30.f
#define ENV_MAX_VBUS_VALUE 200.f
#define ENV_VBUS_HYSTERESIS 3.f


typedef enum
{
    ENV_OK = 0,
    ENV_FAIL,

}env_ret_val_t ;


typedef struct ENVIRONMENT_T
{
    inv_inputs_t * inputs;
    bool transistor_temperature_high;
    bool vbus_in_range;
    bool charger_enabled;
    bool power_connected;
    bool accelerator_error;

    bool order_shutdown;

}env_t;


env_ret_val_t env_init(env_t * instance);

env_ret_val_t env_update(env_t * instance);




void env_overtemperature_rising_handler();
void env_overtemperature_falling_handler();
void env_vbus_out_of_range_handler();
void env_vbus_in_range_handler();

void env_charger_enable();
void env_charger_disable();

