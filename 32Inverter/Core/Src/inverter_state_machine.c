#include "inverter_state_machine.h"
//
//inv_ret_val_t inv_state_machine_update(int * inverter)
//{
//
//}

typedef enum INV_STATUS inverter_status_t;


inv_state_ret_val_t inv_command_state(inv_t * instance, inv_command_t command)
{
    inv_state_ret_val_t return_value = INV_STATE_RET_NO_EFFECT;

    INV_TRANSITION_RET_VAL transition_effect;

    switch (command) {

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
            }

        case INV_COMMAND_CHARGE:

            if(instance->main_status == INV_STATUS_IDLE)
            {
                transition_effect = inv_transition_idle_charge(instance);
            }

            if(transition_effect == 0)
            {
                return_value = INV_STATE_RET_OK;
                instance->main_status = INV_STATUS_CHARGING;
            } else
            {
                log_info("initialized to charge failure");
                return_value = INV_STATE_RET_FAIL;
            }



            break;

        default:
            log_info("Invalid inverter state command");
    }
    return  return_value;
}