#pragma once

#include "iir_filter.h"
#include "stdint.h"
#include "stm32f3xx_hal.h"
#include "math.h"
#include "stdbool.h"
#include "vectors.h"
#include "PID.h"
#include "adc.h"

#define INV_MAX_PWM_PULSE_VAL 2500
#define INV_PID_MAX_OUT 50
#define DEFAULT_CURRENT_FILTER_ALPHA 0.01f
#define INV_MIN_VOLTAGE_VALUE 30.f

typedef struct
{
    uint32_t pin;
    GPIO_TypeDef * port;

}inv_pin_t;

typedef struct
{
    inv_pin_t main_contactor;
    inv_pin_t precharge_contactor;
}inv_io_t;

typedef enum
{
    INV_OK,
    INV_FAIL
}inv_ret_val_t ;

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
    inv_io_t io;
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
    vec_t smooth_set_current;
    inverter_mode_t mode; /**< Control mode requested by user */
    float current_filter_alpha;
    float vbus_filter_alpha;
    vec_t voltage;
    iir_filter_t filter_d;
    iir_filter_t filter_q;

    adcs_t adcs;
} inverter_t;


inv_ret_val_t inv_init(inverter_t *inverter);

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

void inv_slow_tick(inverter_t * inverter);

inv_ret_val_t inv_connect_supply(inverter_t * inverter);
inv_ret_val_t inv_disconnect_supply(inverter_t * inverter);
