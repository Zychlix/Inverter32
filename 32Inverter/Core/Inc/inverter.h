#pragma once

#include "iir_filter.h"
#include "stdint.h"
#include "stm32f3xx_hal.h"
#include "math.h"
#include "stdbool.h"
#include "vectors.h"
#include "PID.h"
#include "adc.h"
#include "inverter_state_machine.h"
#define INV_MAX_PWM_PULSE_VAL 2500
#define INV_PID_MAX_OUT 100
#define DEFAULT_CURRENT_FILTER_ALPHA 0.01f
#define INV_MIN_VOLTAGE_VALUE 30.f

typedef enum
{
    INV_OK,
    INV_FAIL,
    INV_PRECHARGE_FAIL
}inv_ret_val_t ;


typedef enum
{
    INV_STATUS_INITIALIZED = 0,  //Primary state. Not allowed after using inv_init
    INV_STATUS_IDLE,               //Inverter succesfully initialized, awaiting action
} inverter_status_t;

/*
 * Power box state
 */
typedef enum
{
    INV_POWER_DISCONNECTED,
    INV_POWER_PRECHARGE_ENGAGED,
    INV_POWER_ENGAGED
}inverter_power_state_t;

typedef struct
{
    bool transistor_temperature;
    bool motor_temperature;
    bool overcurrent;
    bool low_voltage;
    bool throttle_input_error;
    bool spi_error;
}inverter_error_t;


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

typedef struct {
    SPI_HandleTypeDef *spi_handler;
    float fi;
    float derived_electrical_velocity_rad_s;
    float derived_mechanical_velocity_rad_s; // Mechanical speed. Electrical is this times
} resolver_t;

typedef enum
{
    MODE_AB = 0,
    MODE_DQ,
    MODE_DQ_FREQUENCY,
} inverter_mode_t;

typedef struct
{
    TIM_HandleTypeDef *timer;

    resolver_t resolver;

    ADC_HandleTypeDef *current_adc;
    volatile uint16_t raw_current_adc[2];
    uint16_t current_adc_offset[2];

    float vbus;
    pi_t pid_d;
    pi_t pid_q;
    pi_t pid_a;
    pi_t pid_b;
    vec_t current;
    vec_t set_value; /**< Current requested by the module user, can be in alpha-beta or dq space*/
    vec_t smooth_set_current;
    inverter_mode_t motor_control_mode; /**< Control mode requested by user */
    float current_filter_alpha;
    float vbus_filter_alpha;
    vec_t voltage;



    iir_filter_t filter_d;
    iir_filter_t filter_q;

    inverter_error_t error_flags;
}inv_foc_controller_t;

typedef struct {
    inverter_status_t state;             //Main state machine state

    inv_inputs_t inputs;                //

    bool active;
    bool throttle_control;
    bool voltage_vector_advance;
    inv_io_t relay_box;


    TIM_HandleTypeDef *timer;

    resolver_t resolver;

    ADC_HandleTypeDef *current_adc;
    volatile uint16_t raw_current_adc[2];
    uint16_t current_adc_offset[2];

    float vbus;
    pi_t pid_d;
    pi_t pid_q;
    pi_t pid_a;
    pi_t pid_b;
    vec_t current;
    vec_t set_value; /**< Current requested by the module user, can be in alpha-beta or dq space*/
    vec_t smooth_set_current;
    inverter_mode_t motor_control_mode; /**< Control mode requested by user */
    float current_filter_alpha;
    float vbus_filter_alpha;
    vec_t voltage;

    iir_filter_t filter_d;
    iir_filter_t filter_q;

    inverter_error_t error_flags;

    float frequency_setpoint;

    bool _test_mtpa_control;
    float mtpa_current;

} inv_t;


inv_ret_val_t inv_init(inv_t *inverter);

inv_ret_val_t inv_start(inv_t * inv);
void inv_enable(inv_t *inv, bool status);

void res_read_position(resolver_t *res);

bool inv_get_fault();

void inv_clear_fault();

void inv_set_fault();

void inv_set_pwm(inv_t *inverter, float u, float v, float w);

void inv_tick(inv_t *inverter);

abc_t inv_read_current(inv_t *inverter);

void inv_read_vbus();

int32_t inv_calibrate_current(inv_t *inverter);

void inv_set_mode_and_current(inv_t *inverter, inverter_mode_t mode, vec_t current);

void inv_slow_tick(inv_t * inverter);

inv_ret_val_t inv_connect_supply(inv_t * inverter);
inv_ret_val_t inv_disconnect_supply(inv_t * inverter);
