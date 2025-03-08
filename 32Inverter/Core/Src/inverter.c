//
// Created by michal on 03.05.23.
//

#include <stdio.h>
#include "inverter.h"
#include "resolver.h"
#include "vectors.h"
#include "PID.h"
#include "oscilloscope.h"

#define INV_MIN_VOLTAGE_HYSTERESIS 5.f
#define INV_MAX_TEMPERATURE_DISABLE 70.f //C
#define INV_MAX_TEMPERATURE_ENABLE 65.f //C

void inv_reset_controllers(inverter_t * inverter)
{
    inverter->pid_d.integrated = 0;
    inverter->pid_q.integrated = 0;
    inverter->pid_a.integrated = 0;
    inverter->pid_b.integrated = 0;
}

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

    inverter->current_filter_alpha = DEFAULT_CURRENT_FILTER_ALPHA;

    inverter->filter_d = (iir_filter_t){
        .a = {1.f, -1.93060643f,  0.93293473f},
        .b = {0.00058208f, 0.00116415f, 0.00058208f},
    };
    inverter->filter_q = inverter->filter_d;

    iir_filter_init(&inverter->filter_d);
    iir_filter_init(&inverter->filter_q);


    // current PI
    inverter->pid_d.kp = 1.f;
    inverter->pid_d.ki = 20.f;
    inverter->pid_d.dt = (float) INV_MAX_PWM_PULSE_VAL / (float) SystemCoreClock;
    inverter->pid_d.integrated = 0;
    inverter->pid_d.max_out = INV_PID_MAX_OUT;

    inverter->pid_q.kp = 1.f;
    inverter->pid_q.ki = 20.f;
    inverter->pid_q.dt = (float) INV_MAX_PWM_PULSE_VAL / (float) SystemCoreClock;
    inverter->pid_q.integrated = 0;
    inverter->pid_q.max_out = INV_PID_MAX_OUT;


    inverter->pid_a.kp = 0;
    inverter->pid_a.ki = 1;
    inverter->pid_a.dt = (float) INV_MAX_PWM_PULSE_VAL / (float) SystemCoreClock;
    inverter->pid_a.integrated = 0;
    inverter->pid_a.max_out = INV_PID_MAX_OUT;

    inverter->pid_b.kp = 0;
    inverter->pid_b.ki = 1;
    inverter->pid_b.dt = (float) INV_MAX_PWM_PULSE_VAL / (float) SystemCoreClock;
    inverter->pid_b.integrated = 0;
    inverter->pid_b.max_out = INV_PID_MAX_OUT;
}

uint16_t spi_read_word(SPI_TypeDef *spi)
{
    while (spi->SR & SPI_SR_BSY)
    {
    }
    *(volatile uint16_t*)(&spi->DR) = 0xffff;
    while (!(spi->SR & SPI_SR_RXNE))
    {
    }
    uint16_t data = *(volatile uint16_t*)(&spi->DR);

    return data;
}

void res_read_position(resolver_t *res) {
    // TODO: simplify GPIO toggling
    HAL_GPIO_WritePin(SAMPLE_GPIO_Port, SAMPLE_Pin, 0);
    HAL_GPIO_WritePin(SAMPLE_GPIO_Port, SAMPLE_Pin, 1);

    HAL_GPIO_WritePin(RDVEL_GPIO_Port, RDVEL_Pin, 1);
    HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, 1);
    HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, 0);


    static volatile float resolver_offset = -3.141f;


    uint16_t pos = spi_read_word(res->spi_handler->Instance) >> 4;
    res->fi = (float) pos / 4096.f * 2 * (float) M_PI + resolver_offset;

    HAL_GPIO_WritePin(RDVEL_GPIO_Port, RDVEL_Pin, 0);
    HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, 1);
    HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, 0);
    int16_t speed = (int16_t) (spi_read_word(res->spi_handler->Instance) & 0xfff0) / 16;
    res->velocity = speed * 7; // TODO: rad/s, find a better factor

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
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, true);
    res_read_position(&inverter->resolver);
    vec_t phi = angle(inverter->resolver.fi);

    static volatile abc_t current_abc;
    static volatile vec_t current_ab;
    static volatile vec_t current_dq;
    current_abc = inv_read_current(inverter);
    current_ab = clarkeTransform(current_abc);
    current_dq = parkTransform(current_ab, phi);

    inverter->current.x = iir_filter_calculate(&inverter->filter_d, current_dq.x);
    inverter->current.y = iir_filter_calculate(&inverter->filter_q, current_dq.y);

    // inverter->current.x = (inverter->current_filter_alpha * current_dq.x) + (1.0f - inverter->current_filter_alpha) * inverter->current.x;
    // inverter->current.y = (inverter->current_filter_alpha * current_dq.y) + (1.0f - inverter->current_filter_alpha) * inverter->current.y;

    static volatile float setpoint_alpha = 0.02f;
    inverter->smooth_set_current.x = (setpoint_alpha * inverter->set_current.x) + (1.0f - setpoint_alpha) * inverter->smooth_set_current.x;
    inverter->smooth_set_current.y = (setpoint_alpha * inverter->set_current.y) + (1.0f - setpoint_alpha) * inverter->smooth_set_current.y;

    if (inverter->mode == MODE_DQ)
    {
        static volatile float rotor_flux_linkage = 0.023f; /* Volts / (rad / s) */

        inverter->voltage = (vec_t){
            pid_calc(&inverter->pid_d, inverter->current.x, inverter->smooth_set_current.x),
            pid_calc(&inverter->pid_q, inverter->current.y, inverter->smooth_set_current.y) + rotor_flux_linkage * inverter->resolver.velocity,
        };

        vec_t pwm = {
            inverter->voltage.x / inverter->vbus,
            inverter->voltage.y / inverter->vbus,
        };

        pwm = limit_amplitude(pwm, 1);
        pwm = inverseParkTransform(pwm, phi);
        abc_t pwmABC = inverseClarkeTransform(pwm);
        inv_set_pwm(inverter, pwmABC.a, pwmABC.b, pwmABC.c);
    } else if (inverter->mode == MODE_AB) {
        inverter->voltage = (vec_t){
            pid_calc(&inverter->pid_a, current_ab.x, inverter->set_current.x),
            pid_calc(&inverter->pid_b, current_ab.y, inverter->set_current.y),
        };

        vec_t pwm = {
            inverter->voltage.x / inverter->vbus,
            inverter->voltage.y / inverter->vbus,
        };
        pwm = limit_amplitude(pwm, 1);
        abc_t pwmABC = inverseClarkeTransform(pwm);
        inv_set_pwm(inverter, pwmABC.a, pwmABC.b, pwmABC.c);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, false);
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
    const float amps_per_bit = 1.15f;
    const float symmetry_compensation = 1.00f;
    abc_t result;
    result.c = (float) (inverter->raw_current_adc[1] - inverter->current_adc_offset[1]) * amps_per_bit;
    result.b = symmetry_compensation * (float) (inverter->raw_current_adc[0] - inverter->current_adc_offset[0]) *
               amps_per_bit;
    result.a = -result.b - result.c;

    return result;
}

void inv_enable(inverter_t *inv, bool status) {

    if (status) {
        if(! inv->active)
        {
            HAL_TIM_Base_Start_IT(inv->timer);
            HAL_TIM_PWM_Start(inv->timer, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Start(inv->timer, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(inv->timer, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Start(inv->timer, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(inv->timer, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Start(inv->timer, TIM_CHANNEL_3);
        }

    } else {
        if(inv->active)
        {
            inv_reset_controllers(inv);
            HAL_TIM_Base_Stop(inv->timer);
            HAL_TIM_PWM_Stop(inv->timer, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(inv->timer, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(inv->timer, TIM_CHANNEL_3);
        }

    }
    inv->active = status;
}

void inv_set_mode_and_current(inverter_t *inverter, inverter_mode_t mode, vec_t current)
{
    inverter->mode = mode;
    inverter->set_current = current;


}

void inv_vbus_update(inverter_t * inverter)
{
    float current_vbus = inverter->adcs.vbus;



    if(current_vbus < INV_MIN_VOLTAGE_VALUE)
    {
        inv_enable(inverter,false);
    }
    else
    if(current_vbus > INV_MIN_VOLTAGE_VALUE + INV_MIN_VOLTAGE_HYSTERESIS )
    {
        inverter->vbus = current_vbus;

        inv_enable(inverter,true);
    }
}

void inv_temperature_check(inverter_t * inverter)
{
    if( inverter->adcs.transistor1 > INV_MAX_TEMPERATURE_DISABLE)
    {
        inv_enable(inverter,false);
    } else if ( inverter->adcs.transistor1 < INV_MAX_TEMPERATURE_ENABLE)
    {
        inv_enable(inverter, true);
    }

}

void inv_slow_tick(inverter_t * inverter)
{
    adc4_read(&inverter->adcs);
    adc2_read(&inverter->adcs);


    inv_vbus_update(inverter);



}