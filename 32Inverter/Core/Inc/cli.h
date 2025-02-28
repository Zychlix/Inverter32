#pragma once

void cli_init(UART_HandleTypeDef *huart, inverter_t *inverter);

void cli_poll();