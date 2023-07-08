#pragma once

#include "stdint.h"
#include "stm32f3xx_hal.h"
#include "math.h"
#include "stdbool.h"
#include "vectors.h"
#include "PID.h"

#define INV_MAX_PWM_PULSE_VAL 5000
#define INV_FEEDBACK_CYCLE_DIVISION 2
#define INV_DQ_KP 1
#define INV_DQ_KI 50

typedef struct {
    SPI_HandleTypeDef *spi_handler;
    float fi;
    float velocity;

} resolver_t;

typedef struct {
    TIM_HandleTypeDef *timer;
    resolver_t resolver;
    bool voltage_vector_advance;
    ADC_HandleTypeDef *current_adc;
    volatile uint16_t raw_current_adc[2];
    uint16_t current_adc_offset[2];
    float vbus;
    pi_t pid_d;
    pi_t pid_q;
    vec_t current;
    bool active;
    float current_setpoint;
} inverter_t;


void inv_init(inverter_t *inverter);

void inv_enable(inverter_t *inv, bool status);

void res_read_position(resolver_t *res);

bool inv_get_fault();

void inv_clear_fault();

void inv_set_pwm(inverter_t *inverter, float u, float v, float w);

void inv_tick(inverter_t *inverter);

abc_t inv_read_current(inverter_t *inverter);

void inv_read_vbus();

int32_t inv_calibrate_current(inverter_t *inverter);
