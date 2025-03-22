#pragma once

void cli_init(UART_HandleTypeDef *huart, inverter_t *inverter, chg_t * charger);

void cli_poll();