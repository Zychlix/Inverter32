#include "stdio.h"
#include "inverter.h"
#include "error_log.h"

extern inv_t inv;

void env_overtemperature_rising_handler()
{
    log_info("test_ rising");
    inv_command_state_issue(&inv, INV_COMMAND_IDLE);

}

void env_overtemperature_falling_handler()
{
    log_info("test_ falling");
}

void env_vbus_out_of_range_handler()
{
    log_info("Voltage incorrect!");
    inv_command_state_issue(&inv, INV_COMMAND_IDLE);

    inv.order_shutdown = true;

}
void env_vbus_in_range_handler()
{
    log_info("Voltage correct!");

}


void env_charger_enable()
{
    inv_command_state_issue(&inv,INV_COMMAND_CHARGE);
    log_info("Charger mode enabled");
}
void env_charger_disable()
{
    if(inv.main_status == INV_STATUS_CHARGING)
    {
        inv_command_state_issue(&inv,INV_COMMAND_IDLE);
    }

    log_info("Charger mode disabled");
}


