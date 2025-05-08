#pragma once
#include "fast_data_logger.h"

void cli_init(UART_HandleTypeDef *huart, inv_t *inverter, chg_t * charger, fdl_t *fdl );

void cli_poll();