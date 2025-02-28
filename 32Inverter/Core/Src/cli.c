#include <stdbool.h>
#include <stdio.h>
#include <stm32f3xx.h>
#include <string.h>
#include <inverter.h>

/* PRIVATE VARIABLES */

static struct
{
	UART_HandleTypeDef *huart;
	char command_buffer[64];
	uint8_t buffer_size;
	inverter_t *inverter;
} me;

/* PRIVATE FUNCTIONS */

static void parse_command(char* str)
{
	float arg1, arg2;

	if (strcmp(str, "ping") == 0) {
		printf("Pong!\n");
	} else if(strcmp(str, "ph_curr") == 0) {
		abc_t current = inv_read_current(me.inverter);
		printf("%5.1f %5.1f %5.1f\n", current.a, current.b, current.c);
	} else if(strcmp(str, "ph_curr_raw") == 0) {
		printf("%5d %5d\n", me.inverter->raw_current_adc[0], me.inverter->raw_current_adc[1]);
	}else if (sscanf(str, "AB %f %f", &arg1, &arg2) == 2) {
		printf("AB %f %f\n", arg1, arg2);
	} else {
		printf("Unknown command!\n");
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	me.buffer_size++;
	if (me.buffer_size >= sizeof(me.command_buffer))
	{
		me.buffer_size = 0;
	}

	HAL_UART_Receive_IT(me.huart, (uint8_t*)me.command_buffer + me.buffer_size, 1);
}

/* PUBLIC FUNCTIONS */

void cli_init(UART_HandleTypeDef *huart, inverter_t *inverter)
{
	me.huart = huart;
	me.buffer_size = 0;
	me.inverter = inverter;

	HAL_UART_Receive_IT(me.huart, (uint8_t*)me.command_buffer + me.buffer_size, 1);
}

void cli_poll()
{
	if (me.buffer_size > 0 && me.command_buffer[me.buffer_size - 1] == '\n')
	{
		HAL_UART_AbortReceive(me.huart);
		me.command_buffer[me.buffer_size - 1] = '\0';
		me.buffer_size = 0;
		parse_command(me.command_buffer);
		HAL_UART_Receive_IT(me.huart, (uint8_t*)me.command_buffer + me.buffer_size, 1);
	}
}
