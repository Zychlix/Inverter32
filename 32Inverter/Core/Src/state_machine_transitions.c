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


INV_TRANSITION_RET_VAL inv_transition_idle_charge_ready(inv_t * instance)
{
    if(instance->power_status == INV_POWER_ENGAGED)
    {

        chg_config_filters(instance->charger);
//        instance->charger->setpoint.protection = DEZHOU_BATTERY_OPEN_CHARGING;

        return 0;

    } else

        return -1;
    return 0;
}

INV_TRANSITION_RET_VAL inv_transition_charge_idle(inv_t * instance)
{


    instance->charger->setpoint.protection = DEZHOU_BATTERY_PROTECTION;  //Wait for it to be sent
    chg_deactivate_filters(instance->charger);


    return 0;
}
