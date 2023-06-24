//
// Created by michal on 03.05.23.
//

#include <stdio.h>
#include "inverter.h"
#include "resolver.h"
#include "vectors.h"
#include "PID.h"
#include "oscilloscope.h"

void inv_init(inverter_t *inverter) {
    TIM_OC_InitTypeDef oc_config;

    oc_config.OCMode = TIM_OCMODE_PWM1;
    oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc_config.OCFastMode = TIM_OCFAST_DISABLE;
    oc_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    oc_config.OCIdleState = TIM_OCIDLESTATE_RESET;

    HAL_TIM_PWM_ConfigChannel(inverter->timer, &oc_config, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(inverter->timer, &oc_config, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(inverter->timer, &oc_config, TIM_CHANNEL_3);

    // ADC
    HAL_ADC_Start_DMA(inverter->current_adc, (uint32_t *) &inverter->raw_current_adc, 2);
    HAL_ADC_Start(inverter->current_adc);


    // current PI
    inverter->pid_d.kp = INV_DQ_KP;
    inverter->pid_d.ki = INV_DQ_KI;
    inverter->pid_d.dt = (float) INV_MAX_PWM_PULSE_VAL * INV_FEEDBACK_CYCLE_DIVISION / (float) SystemCoreClock;
    inverter->pid_d.integrated = 0;


    inverter->pid_q.kp = INV_DQ_KP;
    inverter->pid_q.ki = INV_DQ_KI;
    inverter->pid_q.dt = (float) INV_MAX_PWM_PULSE_VAL * INV_FEEDBACK_CYCLE_DIVISION / (float) SystemCoreClock;
    inverter->pid_q.integrated = 0;
}


void res_read_position(resolver_t *res) {
    // TODO: simplify GPIO toggling

    HAL_GPIO_WritePin(SAMPLE_GPIO_Port, SAMPLE_Pin, 0);
    HAL_GPIO_WritePin(SAMPLE_GPIO_Port, SAMPLE_Pin, 1);

    HAL_GPIO_WritePin(RDVEL_GPIO_Port, RDVEL_Pin, 1);
    HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, 1);
    HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, 0);


    const float resolver_offset = -2.90f;


    uint8_t data[2];
    HAL_SPI_Receive(res->spi_handler, data, 1, 10);
    uint16_t pos = ((data[1] << 8) | (data[0])) >> 4;
    res->fi = (float) pos / 4096.f * 2 * (float) M_PI + resolver_offset;


    HAL_GPIO_WritePin(RDVEL_GPIO_Port, RDVEL_Pin, 0);
    HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, 1);
    HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, 0);
    HAL_SPI_Receive(res->spi_handler, data, 1, 10);
    int16_t speed = (int16_t) (((data[1] << 8) | (data[0])) & 0xfff0) / 16;
    res->velocity = speed * 4; // rad/s, find a better factor

    HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, 1);
    HAL_GPIO_WritePin(SAMPLE_GPIO_Port, SAMPLE_Pin, 1);
}

bool inv_get_fault() {
    return HAL_GPIO_ReadPin(FAULT_SR_3V3_GPIO_Port, FAULT_SR_3V3_Pin);
}

void inv_clear_fault() {
    HAL_GPIO_WritePin(FAULT_RST_GPIO_Port, FAULT_RST_Pin, true);
    HAL_Delay(1);
    HAL_GPIO_WritePin(FAULT_RST_GPIO_Port, FAULT_RST_Pin, false);
}

inline float constrain(float x, const float min, const float max) {
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

void inv_set_pwm(inverter_t *inverter, float u, float v, float w) {
    inverter->timer->Instance->CCR1 = INV_MAX_PWM_PULSE_VAL * (0.5 + u / 2.0);
    inverter->timer->Instance->CCR2 = INV_MAX_PWM_PULSE_VAL * (0.5 + v / 2.0);
    inverter->timer->Instance->CCR3 = INV_MAX_PWM_PULSE_VAL * (0.5 + w / 2.0);
}

void inv_tick(inverter_t *inverter) {
    static int i = 0;
    static float t = 0;
    i++;
    if (i % INV_FEEDBACK_CYCLE_DIVISION != 0) return;

    //        if (adcs.vbus < 5) Error_Handler();


    res_read_position(&inverter->resolver);
    vec_t phi = angle(inverter->resolver.fi);
    if (fmod(inverter->resolver.fi, 2 * M_PI) < 0.01) oscilloscope_trig();

    abc_t current_3 = inv_read_current(inverter);
    vec_t current_2 = clarkeTransform(current_3);
    inverter->current = parkTransform(current_2, phi);


    vec_t set_current = {
            0,
            0.2,
    };


    vec_t voltage = {
            pid_calc(&inverter->pid_d, inverter->current.d, set_current.d),
            pid_calc(&inverter->pid_q, inverter->current.q, set_current.q),
    };

    const float flux_linkage = 0.05;
    voltage.q += flux_linkage * inverter->resolver.velocity;

    oscilloscope_push(inverter->current.d, inverter->current.q);
//    oscilloscope_push(current_2.x, current_2.y);
//    oscilloscope_push(current_3.c, current_3.b);


//    vec_t voltage = {
//            0,
//            10,
//    };

    vec_t pwm = {
            voltage.x / inverter->vbus,
            voltage.y / inverter->vbus,
    };

    pwm = limit_amplitude(pwm, 1);


    pwm = inverseParkTransform(pwm, phi);
    abc_t pwmABC = inverseClarkeTransform(pwm);


    inv_set_pwm(inverter, pwmABC.a, pwmABC.b, pwmABC.c);
//    printf("A %6.3f B %6.3f C %6.3f fi %6.3f\r\n", pwmABC.a, pwmABC.b, pwmABC.c, inverter->resolver.fi);
}

int32_t inv_calibrate_current(inverter_t *inverter) {
    if (inverter->active) return -1;

    const uint16_t avg_cycles = 256;
    static uint32_t sum[2];
    sum[0] = 0;
    sum[1] = 0;
    HAL_Delay(1);
    for (int i = 0; i < avg_cycles; i++) {
        sum[0] += inverter->raw_current_adc[0];
        sum[1] += inverter->raw_current_adc[1];
        HAL_Delay(1);
    }
    sum[0] /= avg_cycles;
    sum[1] /= avg_cycles;

    const uint32_t MIN_OFFSET = 1500;
    const uint32_t MAX_OFFSET = 2500;

    if (sum[0] > MAX_OFFSET || sum[0] < MIN_OFFSET) return -1;
    if (sum[1] > MAX_OFFSET || sum[1] < MIN_OFFSET) return -1;

    inverter->current_adc_offset[0] = sum[0];
    inverter->current_adc_offset[1] = sum[1];

    return 0;
}


abc_t inv_read_current(inverter_t *inverter) {
    const float amps_per_bit = 0.1342f;
    const float symmetry_compensation = 1.08f;
    abc_t result;
    result.c = (float) (inverter->raw_current_adc[1] - inverter->current_adc_offset[1]) * amps_per_bit;
    result.b = symmetry_compensation * (float) (inverter->raw_current_adc[0] - inverter->current_adc_offset[0]) *
               amps_per_bit;
    result.a = -result.b - result.c;

    return result;
}

void inv_enable(inverter_t *inv, bool status) {
    if (status) {
        HAL_TIM_Base_Start_IT(inv->timer);
        HAL_TIM_PWM_Start(inv->timer, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(inv->timer, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(inv->timer, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(inv->timer, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(inv->timer, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(inv->timer, TIM_CHANNEL_3);
    } else {
        HAL_TIM_Base_Stop(inv->timer);
        HAL_TIM_PWM_Stop(inv->timer, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(inv->timer, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(inv->timer, TIM_CHANNEL_3);
    }
    inv->active = status;
}