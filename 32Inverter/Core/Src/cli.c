#include <stdbool.h>
#include <stdio.h>
#include <stm32f3xx.h>
#include <string.h>

/* PRIVATE VARIABLES */

static struct
{
	UART_HandleTypeDef *huart;
	char command_buffer[64];
	uint8_t buffer_size;
} me;

/* PRIVATE FUNCTIONS */

static void parse_command(char* str)
{
	float arg1, arg2;

	if (strcmp(str, "ping") == 0) {
		printf("Pong!\n");
	} else if (sscanf(str, "AB %f %f", &arg1, &arg2) == 2) {
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

void cli_init(UART_HandleTypeDef *huart)
{
	me.huart = huart;
	me.buffer_size = 0;

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
