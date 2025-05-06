//
// Created by michal on 17.06.23.
//

#pragma once

#include "stm32f3xx.h"

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

}inv_inputs_t;

typedef struct {
    float vbus;
    float throttleA; //Normalize
    float throttleB;
    float input12V;
    float motor_temp2;

    float transistor1;

} adcs_t;


void adc4_read(inv_inputs_t * inputs);

void adc2_read(inv_inputs_t * inputs);