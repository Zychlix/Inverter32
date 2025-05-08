#pragma once

typedef enum
{
    INV_OK,
    INV_FAIL,
    INV_PRECHARGE_FAIL
}inv_ret_val_t ;


typedef enum
{
    INV_STATUS_INITIALIZED = 0,  //Primary state. Not allowed after using inv_init
    INV_STATUS_IDLE,               //Inverter succesfully initialized, awaiting action
} inverter_status_t;

/*
 * Power box state
 */
typedef enum
{
    INV_POWER_DISCONNECTED,
    INV_POWER_PRECHARGE_ENGAGED,
    INV_POWER_ENGAGED
}inverter_power_state_t;

typedef struct
{
    bool transistor_temperature;
    bool motor_temperature;
    bool overcurrent;
    bool low_voltage;
    bool throttle_input_error;
    bool spi_error;
}inverter_error_t;


inv_ret_val_t inv_state_machine_update(inv_t * inverter);

