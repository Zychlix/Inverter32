#pragma once
#include "stdint.h"
#include "stm32f3xx_hal.h"
#include "math.h"
#include "stdbool.h"

#define INV_MAX_PWM_PULSE_VAL 5000

typedef struct
{
    SPI_HandleTypeDef *spi_handler;
    float  fi;
    float  velocity;

} resolver_t;

typedef struct
{
    TIM_HandleTypeDef * timer;
    resolver_t resolver;
    bool voltage_vector_advance;
} inverter_t;


void inv_init(inverter_t * inverter);

void res_read_position(resolver_t * res);

bool inv_get_fault();

void inv_clear_fault();

void inv_set_pwm(inverter_t *inverter, float u, float v, float w);

void inv_tick(inverter_t *inverter);

