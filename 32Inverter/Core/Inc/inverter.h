#pragma once
#include "stdint.h"
#include "stm32f3xx_hal.h"
#include "math.h"
#include "stdbool.h"

#define INV_PRESCALER 0
#define INV_MAX_PWM_PULSE_VAL 5000
#define INV_DT                0.001

typedef struct
{
    SPI_HandleTypeDef *spi_handler;
    float  fi;
    float  velocity;

} resolver_t;

typedef struct
{
    TIM_HandleTypeDef * timer;

    vector_t voltage_vector;
    vector_t rotor_vector;
    resolver_t resolver;
    bool voltage_vector_advance;
} inverter_t;


void inv_init(inverter_t * inverter);

void res_read_position(resolver_t * res);

bool inv_get_fault();

void inv_clear_fault();

void inv_set_pwm(float u, float v, float w);
