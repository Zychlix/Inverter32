#pragma once
#include "inverter_state_machine_defs.h"
#include "stdint.h"
#include "inverter.h"
#include "dcdc_controller.h"


typedef struct INV inv_t;
typedef struct CHG chg_t;


/*
 * State machine transition handlers
 */
INV_TRANSITION_RET_VAL inv_transition_uninitialized_idle( inv_t * instance);
INV_TRANSITION_RET_VAL inv_transition_idle_drive(inv_t * instance);
INV_TRANSITION_RET_VAL inv_transition_idle_charge_ready(inv_t * instance);
INV_TRANSITION_RET_VAL inv_transition_charge_idle(inv_t * instance);

/*
 * End of transition handlers
 */



inv_state_ret_val_t inv_command_state_issue(inv_t * instance, inv_command_t command);

inv_state_ret_val_t  inv_command_state_executor(inv_t * instance);

inv_state_ret_val_t inv_dispatcher(inv_t * instance);