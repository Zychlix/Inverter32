#pragma once

#include "stdint.h"
#include "stm32f3xx_hal.h"
#include "math.h"
#include "stdbool.h"
#include "vectors.h"
#include "PID.h"

#define INV_MAX_PWM_PULSE_VAL 5000
#define INV_FEEDBACK_CYCLE_DIVISION 2
#define INV_PID_MAX_OUT 10
#define DEFAULT_CURRENT_FILTER_ALPHA 0.995

typedef struct {
    SPI_HandleTypeDef *spi_handler;
    float fi;
    float velocity;
} resolver_t;

typedef enum
{
    MODE_AB = 0,
    MODE_DQ,
} inverter_mode_t;

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
    pi_t pid_a;
    pi_t pid_b;
    vec_t current;
    bool active;
    vec_t set_current; /**< Current requested by the module user, can be in alpha-beta or dq space*/
    inverter_mode_t mode; /**< Control mode requested by user */
    float current_filter_alpha;
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

void inv_set_mode_and_current(inverter_t *inverter, inverter_mode_t mode, vec_t current);
