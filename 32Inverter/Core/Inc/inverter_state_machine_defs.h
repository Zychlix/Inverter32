#pragma once

typedef enum INV_STATUS
{
    INV_STATUS_UNINITIALIZED = 0,  //Primary state. Not allowed after using inv_init
    INV_STATUS_IDLE,               //Inverter succesfully initialized, awaiting action
    INV_STATUS_DRIVE
} inverter_status_t;

typedef enum
{
    INV_STATE_RET_OK,
    INV_STATE_RET_NO_EFFECT,
    INV_STATE_RET_INVALID_TRANSITION,
    INV_STATE_RET_FAIL,

}inv_state_ret_val_t ;      //State machine errors



typedef enum
{
    INV_TRANSITION_OK,
    INV_TRANSITION_FAIL,
    INV_TRANSITION_NO_EFFECT
}inv_transition_ret_val_t;

typedef enum
{
    INV_COMMAND_INITIALIZE = 0,     //Set up the inverter
    INV_COMMAND_DRIVE,              //Inverter in driving mode. PWM activated
    INV_COMMAND_CHARGE,             //Inverter charging
    INV_COMMAND_IDLE,               //Inverter neither charging nor driveing. Safe state

}inv_command_t;

#define INV_TRANSITION_RET_VAL int32_t
