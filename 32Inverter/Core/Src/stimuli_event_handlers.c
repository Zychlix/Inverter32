#include "stdio.h"
#include "inverter.h"
#include "error_log.h"

extern inv_t inv;

void env_overtemperature_rising_handler()
{
    log_info("test_ rising");
}

void env_overtemperature_falling_handler()
{
    log_info("test_ falling");
}

void env_vbus_out_of_range_handler()
{
    log_info("Voltage incorrect!");
    inv.order_shutdown = true;

}
void env_vbus_in_range_handler()
{

}