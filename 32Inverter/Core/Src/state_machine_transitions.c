#include "inverter_state_machine.h"

INV_TRANSITION_RET_VAL inv_transition_uninitialized_idle( inv_t * instance)
{
    inv_start(instance);
    return 0;
}

INV_TRANSITION_RET_VAL inv_transition_idle_drive(inv_t * instance)
{
    if(instance->power_status == INV_POWER_ENGAGED)
    {
        inv_enable(instance,true);
        return 0;
    } else

    return -1;

}

//INV_TRANSITION_RET_VAL inv_transition_drive_idle(inv_t * instance)
//{
//    inv_enable(instance,false);
//    return 0;
//}


INV_TRANSITION_RET_VAL inv_transition_idle_charge(inv_t * instance)
{
    if(instance->power_status == INV_POWER_ENGAGED)
    {
        return 0;
    } else

        return -1;
    return 0;
}

