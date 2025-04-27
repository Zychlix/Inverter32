//
// Created by michal on 03.05.23.
//

#include <stdio.h>
#include "inverter.h"
#include "resolver.h"
#include "vectors.h"
#include "PID.h"
#include "oscilloscope.h"
#include "swo_scope.h"
#include "stm32f3xx_hal_tim_ex.h"

#define INV_MIN_VOLTAGE_HYSTERESIS 5.f
#define INV_MAX_TEMPERATURE_DISABLE 70.f //C
#define INV_MAX_TEMPERATURE_ENABLE 65.f //C

#define TRACE_FREQUENCY_DIVIDER 16

void inv_reset_controllers(inverter_t * inverter)
{
    inverter->pid_d.integrated = 0;
    inverter->pid_q.integrated = 0;
    inverter->pid_a.integrated = 0;
    inverter->pid_b.integrated = 0;
}

void inv_enable_pwm_outputs(inverter_t *inverter, uint32_t Channel)
{
    inverter->timer->Instance->CCER |= (1 | 1<<2)<<Channel;

}

void inv_disable_pwm_outputs(inverter_t *inverter, uint32_t Channel)
{
    inverter->timer->Instance->CCER &= ~(1 | 1<<2)<<Channel;

}


inv_ret_val_t inv_init(inverter_t *inverter) {

    if(inverter->io.main_contactor.pin == 0) return INV_FAIL;
    if(inverter->io.main_contactor.port == 0) return INV_FAIL;
    if(inverter->io.precharge_contactor.pin == 0) return INV_FAIL;
    if(inverter->io.precharge_contactor.port == 0)return INV_FAIL;

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

    HAL_ADC_Start_DMA(inverter->current_adc, (uint32_t *) &inverter->raw_current_adc, 2);

    inverter->current_filter_alpha = DEFAULT_CURRENT_FILTER_ALPHA;

    inverter->filter_d = (iir_filter_t){
        .a = {1.0f, -0.10528975341038055f, 0.07419736034296573f},
        .b = {0.24222690173314645f, 0.4844538034662929f, 0.24222690173314645f},
    };
    inverter->filter_q = inverter->filter_d;

    iir_filter_init(&inverter->filter_d);
    iir_filter_init(&inverter->filter_q);


    // current PI
    inverter->pid_d.kp = 0.8f;
    inverter->pid_d.ki = 200.f;
    inverter->pid_d.dt = (float) INV_MAX_PWM_PULSE_VAL / (float) SystemCoreClock;
    inverter->pid_d.integrated = 0;
    inverter->pid_d.max_out = INV_PID_MAX_OUT;

    inverter->pid_q.kp = 0.8f;
    inverter->pid_q.ki = 200.f;
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

    inv_set_fault();
    inv_start(inverter);

    inverter->state = INV_IDLE;

    return INV_OK;
}

inv_ret_val_t inv_start(inverter_t * inv)
{
    if(!inv)
{
    return INV_FAIL;
}

    HAL_TIM_Base_Start_IT(inv->timer);

    return INV_OK;
}

#define READ_SPI_MAX_CLK_DELAY 72000  //1ms
uint16_t spi_read_word(SPI_TypeDef *spi)
{
    while (spi->SR & SPI_SR_BSY)
    {
    }
    *(volatile uint16_t*)(&spi->DR) = 0xffff;
    uint32_t start_tick = DWT->CYCCNT;
    uint32_t current_tick = DWT->CYCCNT;
    while (!(spi->SR & SPI_SR_RXNE) && (current_tick-start_tick)<READ_SPI_MAX_CLK_DELAY)
    {
        current_tick = DWT->CYCCNT;
    }

    if((current_tick-start_tick)>READ_SPI_MAX_CLK_DELAY)
    {
        Error_Handler();
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

    // static volatile float resolver_offset = -3.141f;
    static volatile float resolver_offset = -2.9f-3.14152f/2+0.1f;

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

void inv_set_fault() {
    HAL_GPIO_WritePin(GPIO_A_GPIO_Port, GPIO_A_Pin, true);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIO_A_GPIO_Port, GPIO_A_Pin, false);
}

inline float constrain(float x, const float min, const float max) {
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

void inv_set_pwm(inverter_t *inverter, float u, float v, float w) {
    inverter->timer->Instance->CCR1 = INV_MAX_PWM_PULSE_VAL * (0.5f + u / 2.0f);
    inverter->timer->Instance->CCR2 = INV_MAX_PWM_PULSE_VAL * (0.5f + v / 2.0f);
    inverter->timer->Instance->CCR3 = INV_MAX_PWM_PULSE_VAL * (0.5f + w / 2.0f);
}

typedef enum {
    MEASURED_CURRENT_D = 1,
    MEASURED_CURRENT_Q,
    FI,
    SET_VOLTAGE_D,
    SET_VOLTAGE_Q,
    CALCULATED_VELOCITY,
    BUS_VOLTAGE
} graph_channel_t;

static void inv_send_trace_data(inverter_t *inverter) {
    static int tick = 0;
    ++tick;

    if (tick > TRACE_FREQUENCY_DIVIDER) {
        tick = 0;

        swo_send_float(MEASURED_CURRENT_D, inverter->current.x);
        swo_send_float(MEASURED_CURRENT_Q, inverter->current.y);
        swo_send_float(FI, inverter->resolver.fi);
        swo_send_float(SET_VOLTAGE_D, inverter->voltage.x);
        swo_send_float(SET_VOLTAGE_Q, inverter->voltage.y);
        swo_send_float(CALCULATED_VELOCITY, inverter->resolver.velocity/100.f);
        swo_send_float(BUS_VOLTAGE, inverter->vbus);
    }
}

void inv_tick(inverter_t *inverter) {
    res_read_position(&inverter->resolver);
    vec_t phi = angle(inverter->resolver.fi);

    static volatile abc_t current_abc;
    static volatile vec_t current_ab;
    static volatile vec_t current_dq;

//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, false);
    current_abc = inv_read_current(inverter);
    current_ab = clarkeTransform(current_abc);
    current_dq = parkTransform(current_ab, phi);

    inverter->current.x = iir_filter_calculate(&inverter->filter_d, current_dq.x);
    inverter->current.y = iir_filter_calculate(&inverter->filter_q, current_dq.y);
    // inverter->current.x = current_dq.x;
    // inverter->current.y = current_dq.y;

    // inverter->current.x = (inverter->current_filter_alpha * current_dq.x) + (1.0f - inverter->current_filter_alpha) * inverter->current.x;
    // inverter->current.y = (inverter->current_filter_alpha * current_dq.y) + (1.0f - inverter->current_filter_alpha) * inverter->current.y;

    static volatile float setpoint_alpha = 0.02f;
    inverter->smooth_set_current.x = (setpoint_alpha * inverter->set_value.x) + (1.0f - setpoint_alpha) * inverter->smooth_set_current.x;
    inverter->smooth_set_current.y = (setpoint_alpha * inverter->set_value.y) + (1.0f - setpoint_alpha) * inverter->smooth_set_current.y;

    if (inverter->mode == MODE_DQ)
    {
        inverter->voltage = (vec_t){
            pid_calc(&inverter->pid_d, inverter->current.x, inverter->smooth_set_current.x),
            pid_calc(&inverter->pid_q, inverter->current.y, inverter->smooth_set_current.y),
        };

        /*inverter->voltage.x = 0;
        inverter->voltage.y = inverter->smooth_set_current.y;*/

        vec_t pwm = {
            inverter->voltage.x / inverter->vbus,
            inverter->voltage.y / inverter->vbus,
        };

        pwm = limit_amplitude(pwm, 1);
        pwm = inverseParkTransform(pwm, phi);
        static volatile abc_t pwmABC = {0};
        pwmABC = inverseClarkeTransform(pwm);
        inv_set_pwm(inverter, pwmABC.a, pwmABC.b, pwmABC.c);
    } else if (inverter->mode == MODE_AB) {
        inverter->voltage = (vec_t){
            pid_calc(&inverter->pid_a, current_ab.x, inverter->set_value.x),
            pid_calc(&inverter->pid_b, current_ab.y, inverter->set_value.y),
        };

        vec_t pwm = {
            inverter->voltage.x / inverter->vbus,
            inverter->voltage.y / inverter->vbus,
        };
        pwm = limit_amplitude(pwm, 1);
        abc_t pwmABC = inverseClarkeTransform(pwm);
        inv_set_pwm(inverter, pwmABC.a, pwmABC.b, pwmABC.c);
    } else if(inverter->mode == MODE_AB_FREQUENCY){
        static float fi = 0;
        fi += 2 * M_PI * inverter->frequency_setpoint / ;
        inverter->voltage = (vec_t){
                sin_lut(fi) * inverter->set_value.x,
                sin_lut(fi) * inverter->set_value.y,
        };

        vec_t pwm = {
                inverter->voltage.x / inverter->vbus,
                inverter->voltage.y / inverter->vbus,
        };
        pwm = limit_amplitude(pwm, 1);
        abc_t pwmABC = inverseClarkeTransform(pwm);
        inv_set_pwm(inverter, pwmABC.a, pwmABC.b, pwmABC.c);
    }

    inv_send_trace_data(inverter);
//    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, false);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, true);
    }
}

int32_t inv_calibrate_current(inverter_t *inverter) {
    if (inverter->active) return -1;

    const uint16_t avg_cycles = 256;
    static uint32_t sum[2];
    sum[0] = 0;
    sum[1] = 0;
    HAL_Delay(1);
    for (int i = 0; i < avg_cycles; i++) {
        HAL_ADC_Start_DMA(inverter->current_adc,sum,2);
        HAL_ADC_PollForConversion(inverter->current_adc, 100);
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
        if(!inv->active)
        {
//            inv_enable_pwm_outputs(inv, TIM_CHANNEL_1);
//            inv_enable_pwm_outputs(inv, TIM_CHANNEL_2);
//            inv_enable_pwm_outputs(inv, TIM_CHANNEL_3);
            HAL_TIM_PWM_Start(inv->timer, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Start(inv->timer, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(inv->timer, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Start(inv->timer, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(inv->timer, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Start(inv->timer, TIM_CHANNEL_3);
            for(int i = 0; i< 300; i++)
            {
                adc4_read(&inv->adcs);
            }
            inv_reset_controllers(inv);
            inv_clear_fault();

        }

    } else {
        if(inv->active)
        {
            inv_set_fault();
            inv_reset_controllers(inv);
//            inv_disable_pwm_outputs(inv, TIM_CHANNEL_1);
//            inv_disable_pwm_outputs(inv, TIM_CHANNEL_2);
//            inv_disable_pwm_outputs(inv, TIM_CHANNEL_3);
            HAL_TIM_PWM_Stop(inv->timer, TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(inv->timer, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(inv->timer, TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(inv->timer, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(inv->timer, TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(inv->timer, TIM_CHANNEL_3);
        }

    }
    inv->active = status;
}

void inv_set_mode_and_current(inverter_t *inverter, inverter_mode_t mode, vec_t current)
{
    inverter->mode = mode;
    inverter->set_value = current;


}

void inv_vbus_update(inverter_t * inverter)
{
    float current_vbus = inverter->adcs.vbus;

    inverter->vbus = current_vbus;

    // if(current_vbus < INV_MIN_VOLTAGE_VALUE)
    // {
    //     inv_enable(inverter,false);
    // }
    // else
    // if(current_vbus > INV_MIN_VOLTAGE_VALUE + INV_MIN_VOLTAGE_HYSTERESIS )
    // {
    //
    //     inv_enable(inverter,true);
    // }
}

void inv_temperature_check(inverter_t * inverter)
{
    if( inverter->adcs.transistor1 > INV_MAX_TEMPERATURE_DISABLE)
    {
        inv_enable(inverter,false);
    } else if ( inverter->adcs.transistor1 < INV_MAX_TEMPERATURE_ENABLE)
    {
//        inv_enable(inverter, true);
    }

}

void inv_slow_tick(inverter_t * inverter)
{
    if(!inverter)
    {
        return;
    }

    if (inverter->state == INV_UNINITIALIZED)
    {
        return;
    }
    adc4_read(&inverter->adcs);
    adc2_read(&inverter->adcs);
    //Make it more human


     inv_vbus_update(inverter);
    inv_temperature_check(inverter);



}


#define INV_PRECHARGE_WAIT_TIME 5000

//Remember to start the ADCS
inv_ret_val_t inv_connect_supply(inverter_t * inverter)
{
    float initial_voltage = inverter->adcs.vbus;
    HAL_GPIO_WritePin(inverter->io.precharge_contactor.port, inverter->io.precharge_contactor.pin, 1);
    printf("Precharge engaged \n");

    uint32_t start_time = HAL_GetTick();
    uint32_t current_time = start_time;

    adc4_read(&inverter->adcs);

    float current_voltage = inverter->adcs.vbus;
    float vbus_derivative = 0;

    while(inverter->adcs.vbus < INV_MIN_VOLTAGE_VALUE && (current_time - start_time)<INV_PRECHARGE_WAIT_TIME) //later change to derivative
    {
        adc4_read(&inverter->adcs);
        vbus_derivative = ((inverter->adcs.vbus - current_voltage)/(float)(HAL_GetTick()-current_time+1));
        current_time = HAL_GetTick();
        current_voltage = inverter->adcs.vbus;
        HAL_Delay(1);

    }


    if(current_time - start_time > INV_PRECHARGE_WAIT_TIME)
    {
        HAL_GPIO_WritePin(inverter->io.precharge_contactor.port, inverter->io.precharge_contactor.pin, 0);
        printf("Contactor engage failed \n");

        return INV_PRECHARGE_FAIL;
    }
    HAL_Delay(500);


   // if(inverter->adcs.vbus>30)
    //{
        HAL_GPIO_WritePin(inverter->io.main_contactor.port, inverter->io.main_contactor.pin, 0);
    adc4_read(&inverter->adcs);


    //}
    HAL_Delay(50);
    HAL_GPIO_WritePin(inverter->io.precharge_contactor.port, inverter->io.precharge_contactor.pin, 0);
    HAL_Delay(100);

    return INV_OK;
}
inv_ret_val_t inv_disconnect_supply(inverter_t * inverter)
{

    HAL_GPIO_WritePin(inverter->io.main_contactor.port, inverter->io.main_contactor.pin, 1);
    HAL_Delay(100);
    HAL_GPIO_WritePin(inverter->io.precharge_contactor.port, inverter->io.precharge_contactor.pin, 0);
    printf("Contactor disengaged\n");

    HAL_Delay(500);

    return INV_OK;
}