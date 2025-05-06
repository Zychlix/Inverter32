//
// Created by michal on 17.06.23.
//

#pragma once

#include "stm32f3xx.h"

typedef struct {
    float vbus;
    float throttleA; //Normalize
    float throttleB;
    float input12V;
    float motor_temp2;

    float transistor1;
    ADC_HandleTypeDef *adc4;
    ADC_HandleTypeDef *adc2;
} adcs_t;


void adc4_read(adcs_t *result);

void adc2_read(adcs_t * result);