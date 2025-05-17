#include "inverter_state_machine.h"

INV_TRANSITION_RET_VAL inv_transition_uninitialized_idle( inv_t * instance)
{
    inv_start(instance);
    return 0;
}

INV_TRANSITION_RET_VAL inv_transition_idle_drive(inv_t * instance)
{


}

INV_TRANSITION_RET_VAL inv_transition_idle_charge(inv_t * instance)
{

}

