#include "stdio.h"
#include "inverter.h"
#include "error_log.h"

extern inv_t inv;

void env_overtemperature_rising_handler()
{
    log_info("Overtemperature!");
    inv_command_state_issue(&inv, INV_COMMAND_IDLE);

}

void env_overtemperature_falling_handler()
{
    log_info("Overtemperature cancelled");
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
    inv_command_state_issue(&inv, INV_COMMAND_CHARGE_READY);
    log_info("Charger switch enabled");
}
void env_charger_disable()
{
    if(inv.main_status == INV_STATUS_CHARGE_READY)
    {
        inv_command_state_issue(&inv,INV_COMMAND_IDLE);
    }

    log_info("Charger switch disabled");
}


void env_charger_can_stream_start()
{
    log_info("Charger started sending CAN messages");
}

void env_charger_can_stream_stop()
{
    log_info("Charger stopped sending CAN messages");
}