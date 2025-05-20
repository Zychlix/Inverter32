//
// Created by michal on 17.06.23.
//

#pragma once

#include "stm32f3xx.h"
#include "stdbool.h"


#define BRK_IN_PIN GPIO_PIN_13
#define BRK_IN_PORT GPIOD

typedef struct
{
    float bus_voltage;
    float supply_voltage;
    float throttle_a_voltage;
    float throttle_b_voltage;

    float motor_A_temperature;
    float motor_B_temperature;

    float igbt_A_temperature;
    float igbt_B_temperature;
    float igbt_C_temperature;

    ADC_HandleTypeDef *adc4;
    ADC_HandleTypeDef *adc2;

    bool igbt_temperature_high;

    bool charger_switch;    //Charger enabled

}inv_inputs_t;



void adc4_read(inv_inputs_t * inputs);

void adc2_read(inv_inputs_t * inputs);

void io_read(inv_inputs_t * inputs);