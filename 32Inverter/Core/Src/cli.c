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
	char c;

	if (strcmp(str, "ping") == 0) {
		printf("Pong!\n");
	} else if(strcmp(str, "curr") == 0) {
		abc_t current = inv_read_current(me.inverter);
		printf("%5.1f %5.1f %5.1f\n", current.a, current.b, current.c);
	} else if(strcmp(str, "curr_raw") == 0)
	{
		printf("%5d %5d\n", me.inverter->raw_current_adc[0], me.inverter->raw_current_adc[1]);
	} else if(strcmp(str, "curr_ab") == 0) {
		vec_t current = clarkeTransform(inv_read_current(me.inverter));
		printf("A %5.1f B %5.1f\n", current.x, current.y);
	} else if (sscanf(str, "ab %f %f", &arg1, &arg2) == 2) {
		/* Set current in alpha beta control mode
		 * ab <current amplitude [A]> <phase [rad]> */

		vec_t current = angle(arg2);
		current.x *= arg1;
		current.y *= arg1;

		printf("A %f B %f\n", current.x, current.y);
		inv_set_mode_and_current(me.inverter, MODE_AB, current);
	} else if (sscanf(str, "pi %c %f %f", &c, &arg1, &arg2) == 3) {
		/* Sets PI coefficients of current controllers
		 * pi <controler name> <proportional> <integral>
		 * controller name:
		 *	'a' alpha beta mode
		 *	'd' d q mode
		 */

		if (c == 'a')
		{
			me.inverter->pid_a.kp = arg1;
			me.inverter->pid_a.ki = arg2;
			me.inverter->pid_b.kp = arg1;
			me.inverter->pid_b.ki = arg2;
		} else if (c == 'd') {
			me.inverter->pid_d.kp = arg1;
			me.inverter->pid_d.ki = arg2;
			me.inverter->pid_q.kp = arg1;
			me.inverter->pid_q.ki = arg2;
		} else {
			printf("Unknown controller %c\n", c);
		}
	} else if(strcmp(str, "res_angle") == 0) {
		printf("%5.3f\n", me.inverter->resolver.fi);
	}else {
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
	HAL_UART_Receive_IT(me.huart, (uint8_t*)me.command_buffer + me.buffer_size, 1);
	if (me.buffer_size > 0 && me.command_buffer[me.buffer_size - 1] == '\n')
	{
		HAL_UART_AbortReceive(me.huart);
		me.command_buffer[me.buffer_size - 1] = '\0';
		me.buffer_size = 0;
		parse_command(me.command_buffer);
		HAL_UART_Receive_IT(me.huart, (uint8_t*)me.command_buffer + me.buffer_size, 1);
	}
}
