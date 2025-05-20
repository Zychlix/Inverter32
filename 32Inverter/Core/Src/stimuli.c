#include "stimuli.h"
#include "stdio.h"

env_ret_val_t env_init(env_t * instance)
{
    if(!instance)
    {
        return ENV_FAIL;
    }

    if(!instance->inputs)
    {
        return ENV_FAIL;
    }


    return  ENV_OK;
}

void env_temperature_check(env_t * instance)
{
   bool temperature_too_high_new = instance->inputs->igbt_A_temperature > INV_MAX_TEMPERATURE_DISABLE;

    if(!instance->transistor_temperature_high && temperature_too_high_new) //Transistor overtemperature
    {
        instance->transistor_temperature_high = true;
        env_overtemperature_rising_handler();
    }

    if(instance->transistor_temperature_high && !temperature_too_high_new)
    {
        instance->transistor_temperature_high = false;
        env_overtemperature_falling_handler();
    }


}

void env_voltage_check(env_t * instance)
{
    //Add Hysterisis!!!
    float vbus = instance->inputs->bus_voltage;
    if(instance->vbus_in_range)
    {
        //If vbus is correct
        if(vbus< (ENV_MIN_VBUS_VALUE-ENV_VBUS_HYSTERESIS) || vbus> (ENV_MAX_VBUS_VALUE+ENV_VBUS_HYSTERESIS))
        {
            instance->vbus_in_range = 0;
            env_vbus_out_of_range_handler();
        }

    } else
    {
        if(vbus> (ENV_MIN_VBUS_VALUE+ENV_VBUS_HYSTERESIS) && vbus< (ENV_MAX_VBUS_VALUE-ENV_VBUS_HYSTERESIS))
        {
            instance->vbus_in_range = 1;
            env_vbus_in_range_handler();
        }
    }
}

void env_charger_check(env_t * instance)
{
    bool charger_state_new = instance->inputs->charger_switch;

    if(instance->charger_enabled)
    {
        if(!charger_state_new)
        {
            env_charger_enable();
            instance->charger_enabled = false;
            //If button was depressed
        }
    } else
    {
        if(charger_state_new)
        {
            env_charger_disable();
            instance->charger_enabled = true;

            //If button was depressed
        }
    }
}

env_ret_val_t  env_update(env_t * instance)
{
    if(!instance)
    {
        return ENV_FAIL;
    }

    if(!instance->inputs)
    {
        return ENV_FAIL;
    }

    env_temperature_check(instance);

    env_voltage_check(instance);

    env_charger_check(instance);

    return ENV_OK;



}

