#include "inverter_state_machine.h"


typedef enum INV_STATUS inverter_status_t;

inv_state_ret_val_t inv_command_state_issue(inv_t * instance, inv_command_t command)
{
    if(!instance)
        return INV_STATE_RET_FAIL;
    if(instance->current_command != INV_COMMAND_NO_COMMAND)
    {
        return INV_STATE_COMMAND_BUFFER_FULL;
    }

    instance->current_command = command;

    return INV_STATE_RET_OK;
}

inv_state_ret_val_t inv_command_state_executor(inv_t * instance)
{
    inv_state_ret_val_t return_value = INV_STATE_RET_NO_EFFECT;

    INV_TRANSITION_RET_VAL transition_effect;

    switch (instance->current_command) {

        case INV_COMMAND_INITIALIZE:
            /*
            * On Transition from uninitialized to initialized
            */
            if(instance->main_status == INV_STATUS_UNINITIALIZED)
            {
                transition_effect = inv_transition_uninitialized_idle(instance);

                if(transition_effect == 0)
                {
                    return_value = INV_STATE_RET_OK;
                    instance->main_status = INV_STATUS_IDLE;
                } else
                {
                    log_info("uninitialized to initialized transition failure");
                    return_value = INV_STATE_RET_FAIL;
                }
            }
            instance->current_command = INV_COMMAND_NO_COMMAND;
            break;


        case INV_COMMAND_DRIVE:
            /*
             * On Transition from initialized to drive
             */
            if(instance->main_status == INV_STATUS_IDLE)
            {
                transition_effect = inv_transition_idle_drive(instance);

                if(transition_effect == 0)
                {
                    return_value = INV_STATE_RET_OK;
                    instance->main_status = INV_STATUS_DRIVE;
                } else
                {
                    log_info("initialized to drive failure");
                    return_value = INV_STATE_RET_FAIL;
                }
            } else
            {
                log_info("Inverter not in idle. Cannot proceed to DRIVE");

            }
            instance->current_command = INV_COMMAND_NO_COMMAND;
            break;

        case INV_COMMAND_CHARGE_READY:

            if(instance->main_status == INV_STATUS_IDLE || 1) //TODO quick hack
            {
                transition_effect = inv_transition_idle_charge_ready(instance);

                if(transition_effect == 0)
                {
                    return_value = INV_STATE_RET_OK;
                    instance->main_status = INV_STATUS_CHARGE_READY;
                }
                else
                {
                    log_info("initialized to charge failure");
                    return_value = INV_STATE_RET_FAIL;
                }
            }
            else
            {
                log_info("Inverter not in Idle");
                return_value = INV_STATE_RET_INVALID_TRANSITION;
            }
            instance->current_command = INV_COMMAND_NO_COMMAND;
            break;

        case INV_COMMAND_IDLE:
            inv_enable(instance, false);
            if(instance->main_status == INV_STATUS_CHARGE_READY)
            {
                inv_transition_charge_idle(instance);
            }

            instance->main_status = INV_STATUS_IDLE;
            instance->current_command = INV_COMMAND_NO_COMMAND;
            break;
        case INV_COMMAND_NO_COMMAND:
            break;

        default:
            log_info("Invalid inverter state command");
    }


    return  return_value;
}

inv_state_ret_val_t inv_dispatcher(inv_t * instance)
{
    if(instance->order_shutdown)
    {
        instance->order_shutdown = false;
        inv_disconnect_supply(instance);
        inv_enable(instance,false);
    }
}