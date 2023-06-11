#pragma once
#include "stdint.h"
#include "stm32f3xx_hal.h"
#include "math.h"
#include "stdbool.h"

#define INV_PRESCALER 0
#define INV_MAX_PWM_PULSE_VAL 5000
#define INV_DT                0.001

typedef struct VECTOR
{
    double argument; // Rad <0;2pi>
    double value; // |v| <0;1>

} vector_t;

typedef struct POLE
{
    TIM_HandleTypeDef * timer_handler;
    uint32_t channel;
    uint16_t width;
} pole_t;

typedef struct RESOLVER
{
    SPI_HandleTypeDef *spi_handler;
    float  fi;
    float  velocity;

} resolver_t;

typedef struct INVERTER
{
    TIM_OC_InitTypeDef oc_config;

    pole_t poles[3];
    vector_t voltage_vector;
    vector_t rotor_vector;
    resolver_t resolver;
    bool voltage_vector_advance;


} inverter_t;


void inv_init(inverter_t * inverter);

void set_pulses(inverter_t *  inverter);

void inv_set_power(inverter_t* inverter, uint8_t phase_number, int32_t power); //-+ 1000 is the way to go

void inv_voltage_vector_apply(inverter_t * inverter, vector_t * vector);

void res_read_position(resolver_t * res);

bool inv_get_fault();

void inv_clear_fault();