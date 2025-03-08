//
// Created by michal on 17.06.23.
//

#include "adc.h"
#include "math.h"


const float VREF = 3.3f;
const float NTC_V_HIGH = 5.0f;
const float NTC_BETA = 3500; // TODO: CZY MOŻE 3300?
const float NTC_R0_IGBT = 5e3;
const float NTC_R0_MOTOR = 10e3;
const float NTC_R_LOWSIDE_IGBT = 680;
const float NTC_R_LOWSIDE_MOTOR = 1e3f;
const float T_ABSOLUTE_ZERO = 273;

float convert_ntc(uint16_t raw, float r0_ntc, float r_lowside){
    float adc_voltage = (VREF * (float)raw/ 4095.f);
    float ntc_resistance = r_lowside * (NTC_V_HIGH / adc_voltage - 1);
    float temperature = 1 / (logf(ntc_resistance / r0_ntc) / NTC_BETA + 1 / (T_ABSOLUTE_ZERO+25)) - T_ABSOLUTE_ZERO;
    return temperature;
}

void adc4_read(adcs_t *adcs){
    static uint16_t data[5];
    HAL_ADC_Start_DMA(adcs->adc4, (uint32_t *) data, 5);
    HAL_ADC_PollForConversion(adcs->adc4, 10);

    adcs->motor_temp2 = convert_ntc(data[0],NTC_R0_MOTOR, NTC_R_LOWSIDE_MOTOR);
    adcs->throttleA = (float)data[1] / 4095.0f;
    adcs->throttleB = (float)data[2] / 4095.0f;

    const float VBUS_VOLTS_PER_BIT = 1.0f/7.0f;
    const uint16_t VBUS_OFFSET = 480;

    const float vbus_alpha = 0.005f;

    static volatile float new_vbus;
    new_vbus = (float)((int16_t)data[3] - VBUS_OFFSET) * VBUS_VOLTS_PER_BIT;
    adcs->vbus = vbus_alpha  *  new_vbus + (1-vbus_alpha) * adcs->vbus;


    const float INPUT_12V_VOLTS_PER_BIT = VREF / 4095.f * 11;
    adcs->input12V = (float)data[4] * INPUT_12V_VOLTS_PER_BIT;
}

void adc2_read(adcs_t * result)
{
    HAL_ADC_Start(result->adc2);
    HAL_ADC_PollForConversion(result->adc2,10);
    result->transistor1 = convert_ntc(HAL_ADC_GetValue(result->adc2), NTC_R0_IGBT,NTC_R_LOWSIDE_IGBT);
}