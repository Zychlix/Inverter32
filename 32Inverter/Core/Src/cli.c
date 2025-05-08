#include <stdbool.h>
#include <stdio.h>
#include <stm32f3xx.h>
#include <string.h>
#include <inverter.h>
#include <dcdc_controller.h>

/* PRIVATE VARIABLES */

static struct
{
	UART_HandleTypeDef *huart;
	char command_buffer[64];
	uint8_t buffer_size;
	inv_t *inverter;
    chg_t * charger;
} me;

/* PRIVATE FUNCTIONS */

static void parse_command(char* str)
{
	float arg1, arg2, arg3;
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
	} else if(strcmp(str, "curr_dq") == 0) {
		printf("d %5.1f q %5.1f\n", me.inverter->current.x, me.inverter->current.y);
	}else if (sscanf(str, "ab %f %f", &arg1, &arg2) == 2) {
		/* Set current in alpha beta control mode
		 * ab <current amplitude [A]> <phase [rad]> */

		vec_t current = angle(arg2);
		current.x *= arg1;
		current.y *= arg1;

		printf("A %f B %f\n", current.x, current.y);
		inv_set_mode_and_current(me.inverter, MODE_AB, current);
	} else if (sscanf(str, "dq %f %f", &arg1, &arg2) == 2) {
		/* Set current in alpha beta control mode
		 * dq <d current [A]> <q current [A]> */

		vec_t current = {arg1, arg2};

		printf("ok\n");
		inv_set_mode_and_current(me.inverter, MODE_DQ, current);
	}else if (sscanf(str, "pi %c %f %f", &c, &arg1, &arg2) == 3) {
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
			printf("ok\n");
		} else if (c == 'd') {
			me.inverter->pid_d.kp = arg1;
			me.inverter->pid_d.ki = arg2;
			me.inverter->pid_q.kp = arg1;
			me.inverter->pid_q.ki = arg2;
			printf("ok\n");
		} else {
			printf("Unknown controller %c\n", c);
		}
	} else if(strcmp(str, "res_angle") == 0) {
		printf("%5.3f\n", me.inverter->resolver.fi);
    } else if(strcmp(str, "engage") == 0) {
        inv_connect_supply(me.inverter);
    } else if(strcmp(str, "disengage") == 0) {
        inv_disconnect_supply(me.inverter);

    } else if(strcmp(str, "enable") == 0) {
        inv_enable(me.inverter,true);
        printf("ok\n");

    } else if(strcmp(str, "disable") == 0) {
        inv_enable(me.inverter,false);
        printf("ok\n");

    } else if(strcmp(str, "voltage") == 0) {
        printf("voltage: %fV \n", me.inverter->vbus);

    } else if(strcmp(str, "temp") == 0) {
        printf("Transistor A temp: %f C \n",me.inverter->inputs.igbt_A_temperature);
        printf("Motor B temp: %f C UNCORRECTED\n",me.inverter->inputs.motor_B_temperature);

    }else if (sscanf(str, "throttle %c", &c) == 1) {
        /* Sets PI coefficients of current controllers
         * pi <controler name> <proportional> <integral>
         * controller name:
         *	'a' alpha beta mode
         *	'd' d q mode
         */

        if (c == '1')
        {
            me.inverter->throttle_control = true;
            printf("Throttle control enabled \n");
        } else
        {
            me.inverter->throttle_control = false;
            printf("Throttle control disabled \n");

        }
        }else if (sscanf(str, "charger %c %f", &c, &arg1) == 2) {
        /*Charger control
         *
         *
         */

        if (c == 'v')
        {

            me.charger->setpoint.voltage = arg1;
        }

        if (c == 'a')
        {

            me.charger->setpoint.current = arg1;
        }
        if(c == 'm')
        {
            if(arg1>0)
            {
                me.charger->setpoint.mode = DEZHOU_MODE_CHARGING;
            } else
            {
                me.charger->setpoint.mode = DEZHOU_MODE_HEATING;
            }
        }

        if(c == 'p')
        {
            if(arg1>0)
            {
                me.charger->setpoint.protection = DEZHOU_BATTERY_OPEN_CHARGING;
            } else
            {
                me.charger->setpoint.protection = DEZHOU_BATTERY_PROTECTION;
            }
        }


        printf("OK \n");
    } else if(strcmp(str, "chg_info") == 0) {
        chg_print_data(me.charger);
    } else if(sscanf(str, "vf %f %f %f", &arg1, &arg2, &arg3) == 3) {
        /* Output voltage with a given frequency  in DQ mode
         * Arguments:
         *  - frequency [Hz]
         *  - voltage amplitude D [V]
         *  - voltage amplitude Q [V]
         */

        me.inverter->set_value.x = arg2;
        me.inverter->set_value.y = arg3;

        me.inverter->frequency_setpoint = arg1;

        me.inverter->motor_control_mode = MODE_DQ_FREQUENCY;

        printf("OK \n");





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

void cli_init(UART_HandleTypeDef *huart, inv_t *inverter, chg_t * charger)
{
	me.huart = huart;
	me.buffer_size = 0;
	me.inverter = inverter;
	me.charger = charger;

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
