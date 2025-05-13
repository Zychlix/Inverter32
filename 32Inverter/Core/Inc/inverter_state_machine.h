#pragma once
#include "inverter_state_machine_defs.h"
#include "stdint.h"
#include "inverter.h"

typedef struct INV inv_t;



/*
 * State machine transition handlers
 */
INV_TRANSITION_RET_VAL inv_transition_uninitialized_idle( inv_t * instance);
INV_TRANSITION_RET_VAL inv_transition_idle_drive(inv_t * instance);

/*
 * End of transition handlers
 */


inv_state_ret_val_t inv_command_state(inv_t * instance, inv_command_t command);