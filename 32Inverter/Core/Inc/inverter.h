#pragma once

#include "inverter_state_machine.h"
#include "iir_filter.h"
#include "stdint.h"
#include "stm32f3xx_hal.h"
#include "math.h"
#include "stdbool.h"
#include "vectors.h"
#include "PID.h"
#include "adc.h"
#include "error_log.h"
#include "stimuli.h"
#include "dcdc_controller.h"

#define INV_MAX_PWM_PULSE_VAL 2500
#define INV_PID_MAX_OUT 100
#define DEFAULT_CURRENT_FILTER_ALPHA 0.01f



typedef enum
{
    INV_OK,
    INV_FAIL,
    INV_PRECHARGE_FAIL
}inv_ret_val_t ;



/*
 * Power box state
 */
typedef enum
{
    INV_POWER_DISCONNECTED = 0 ,
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

typedef struct INV {
    inverter_status_t main_status;                //Main state machine state
    inverter_power_state_t power_status;          //Power box status
    inv_command_t current_command;

    inv_inputs_t inputs;                    //Input structure

    bool order_shutdown;
    bool active;
    bool throttle_control;
    bool adc_readings_ready;
    bool voltage_vector_advance;
    inv_io_t relay_box;                     //Relay IO


    TIM_HandleTypeDef *timer;               //Main PWM timer Handler

    resolver_t resolver;                    //Resolver

    ADC_HandleTypeDef *current_adc;         //Current ADC Handler
    volatile uint16_t raw_current_adc[2];   //Raw current A B
    uint16_t current_adc_offset[2];         //Current raw zero


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
    float mtpa_setpoint;

    env_t stimuli;

    chg_t *charger;
} inv_t;



void res_init(resolver_t * res);

void res_read_position(resolver_t *res);


inv_ret_val_t inv_init(inv_t *inverter);

inv_ret_val_t inv_start(inv_t * inv);

void inv_enable(inv_t *inverter, bool status);


bool inv_get_fault();

void inv_clear_fault();

void inv_set_fault();


void inv_set_pwm(inv_t *inverter, float u, float v, float w);

abc_t inv_read_current(inv_t *inverter);

int32_t inv_calibrate_current(inv_t *inverter);

void inv_read_vbus();

void inv_set_mode_and_current(inv_t *inverter, inverter_mode_t mode, vec_t current);



void inv_pwm_tick(inv_t *inverter);

void inv_auxiliary_tick(inv_t * inverter);

inv_ret_val_t inv_connect_supply(inv_t * inverter);
inv_ret_val_t inv_disconnect_supply(inv_t * inverter);

float inv_get_throttle(inv_t * inverter);
