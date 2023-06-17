//
// Created by michal on 03.05.23.
//

#include <stdio.h>
#include "inverter.h"
#include "resolver.h"
#include "vectors.h"

void inv_init(inverter_t * inverter)
{
    TIM_OC_InitTypeDef oc_config;

    oc_config.OCMode       = TIM_OCMODE_PWM1;
    oc_config.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc_config.OCFastMode   = TIM_OCFAST_DISABLE;
    oc_config.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    oc_config.OCIdleState  = TIM_OCIDLESTATE_RESET;

    HAL_TIM_Base_Start_IT(inverter->timer);

    HAL_TIM_PWM_ConfigChannel(inverter->timer, &oc_config, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(inverter->timer, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(inverter->timer, TIM_CHANNEL_1);

    HAL_TIM_PWM_ConfigChannel(inverter->timer, &oc_config, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(inverter->timer, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(inverter->timer, TIM_CHANNEL_2);

    HAL_TIM_PWM_ConfigChannel(inverter->timer, &oc_config, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(inverter->timer, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(inverter->timer, TIM_CHANNEL_3);

}

void res_read_position(resolver_t * res)
{
    // TODO: simplify GPIO toggling
    HAL_GPIO_WritePin(RD_GPIO_Port,RD_Pin,1);
    HAL_GPIO_WritePin(RDVEL_GPIO_Port,RDVEL_Pin,1);
    HAL_GPIO_WritePin(SAMPLE_GPIO_Port,SAMPLE_Pin,1);

    const float resolver_offset = -3 * (float)M_PI / 4;

    HAL_GPIO_WritePin(RD_GPIO_Port,RD_Pin,0);
    HAL_GPIO_WritePin(SAMPLE_GPIO_Port,SAMPLE_Pin,0);

    uint8_t data[2];
    HAL_SPI_Receive(res->spi_handler,data,1,10);
    HAL_GPIO_WritePin(RD_GPIO_Port,RD_Pin,1);
    uint16_t pos = ((data[1]<<8) | (data[0]))>>4;
    res->fi = (float)pos / 4096.f * 2 * (float)M_PI + resolver_offset;

    HAL_GPIO_WritePin(RD_GPIO_Port,RD_Pin,0);
    HAL_SPI_Receive(res->spi_handler,data,1,10);
    HAL_GPIO_WritePin(RDVEL_GPIO_Port,RDVEL_Pin,0);

    int16_t speed = ((data[1]<<8) | (data[0]))>>4;  // TODO: simplify
//    if((data[1]<<8)&1<<5)
//    {
//        speed = - speed;
//    }
    res->velocity= speed;

    HAL_GPIO_WritePin(RD_GPIO_Port,RD_Pin,1);
    HAL_GPIO_WritePin(SAMPLE_GPIO_Port,SAMPLE_Pin,1);
}

bool inv_get_fault(){
    return HAL_GPIO_ReadPin(FAULT_SR_3V3_GPIO_Port, FAULT_SR_3V3_Pin);
}

void inv_clear_fault(){
    HAL_GPIO_WritePin(FAULT_RST_GPIO_Port, FAULT_RST_Pin, true);
    HAL_Delay(1);
    HAL_GPIO_WritePin(FAULT_RST_GPIO_Port, FAULT_RST_Pin, false);
}

void inv_set_pwm(inverter_t *inverter, float u, float v, float w){
    inverter->timer->Instance->CCR1 = INV_MAX_PWM_PULSE_VAL * (0.5 + u / 2.0);
    inverter->timer->Instance->CCR2 = INV_MAX_PWM_PULSE_VAL * (0.5 + v / 2.0);
    inverter->timer->Instance->CCR3 = INV_MAX_PWM_PULSE_VAL * (0.5 + w / 2.0);
}

void inv_tick(inverter_t *inverter){
    res_read_position(&inverter->resolver);

    vec_t phi = angle(inverter->resolver.fi + (float)M_PI / 2);
//    vec_t phi = angle(0);

    const float amplitude = 0.1f;

    vec_t pwm = {
            amplitude,
            0,
    };

    pwm = parkTransform(pwm, phi);
    abc_t pwmABC = inverseClarkeTransform(pwm);

    inv_set_pwm(inverter, pwmABC.a, pwmABC.b, pwmABC.c);
    printf("A %.3f B %.3f C %.3f fi %6.3f\r\n", pwmABC.a, pwmABC.b, pwmABC.c, inverter->resolver.fi);
}