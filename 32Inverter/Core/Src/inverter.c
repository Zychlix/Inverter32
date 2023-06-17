//
// Created by michal on 03.05.23.
//

#include "inverter.h"
#include "resolver.h"

void inv_init(inverter_t * inverter)
{
    TIM_OC_InitTypeDef oc_config;

    inverter->oc_config.OCMode       = TIM_OCMODE_PWM1;
    inverter->oc_config.OCPolarity   = TIM_OCPOLARITY_HIGH;
    inverter->oc_config.OCFastMode   = TIM_OCFAST_DISABLE;
    inverter->oc_config.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    inverter->oc_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    inverter->oc_config.OCIdleState  = TIM_OCIDLESTATE_RESET;

    HAL_TIM_Base_Start_IT(inverter->poles[0].timer_handler);
    for(int i = 0 ; i< 3; i++)
    {
        HAL_TIM_PWM_ConfigChannel(inverter->poles[i].timer_handler, &inverter->oc_config, inverter->poles[i].channel);
        HAL_TIM_PWM_Start(inverter->poles[i].timer_handler, inverter->poles[i].channel);
        HAL_TIMEx_PWMN_Start(inverter->poles[i].timer_handler, inverter->poles[i].channel);
    }
}

void res_read_position(resolver_t * res)
{

    const float resolver_offset = 7.0 / 4.0 * M_PI;

    HAL_GPIO_WritePin(RD_RES_PORT,RD_RES_PIN,0);
    HAL_GPIO_WritePin(SAMPLE_RES_PORT,SAMPLE_RES_PIN,0);

    //HAL_Delay(1);
    uint8_t data[2];
    HAL_SPI_Receive(res->spi_handler,data,1,10);
    HAL_GPIO_WritePin(RD_RES_PORT,RD_RES_PIN,1);
    uint16_t pos = ((data[1]<<8) | (data[0]))>>4;
    res->fi = pos / 4096.f * 2 * M_PI - resolver_offset;

    HAL_GPIO_WritePin(RD_RES_PORT,RD_RES_PIN,0);
    HAL_SPI_Receive(res->spi_handler,data,1,10);
    HAL_GPIO_WritePin(RDVEL_GPIO_Port,RDVEL_RES_PIN,0);
    int16_t speed = ((data[1]<<8) | (data[0]))>>4;
//    if((data[1]<<8)&1<<5)
//    {
//        speed = - speed;
//    }
    res->velocity= speed;

    HAL_GPIO_WritePin(RD_RES_PORT,RD_RES_PIN,1);
    HAL_GPIO_WritePin(SAMPLE_RES_PORT,SAMPLE_RES_PIN,1);

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