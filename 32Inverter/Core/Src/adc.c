//
// Created by michal on 17.06.23.
//

#include "adc.h"
#include "math.h"


const float VREF = 3.3f;
const float NTC_V_HIGH = 5.0f;
const float NTC_BETA = 3500; // TODO: CZY MOŻE 3300?
const float NTC_R0 = 10e3;
const float NTC_R_LOWSIDE = 4.7e3;
const float T_ABSOLUTE_ZERO = 273;

float convert_ntc(uint16_t raw){
    float adc_voltage = (VREF * (float)raw/ 4095.f);
    float ntc_resistance = NTC_R_LOWSIDE * (NTC_V_HIGH / adc_voltage - 1);
    float temperature = 1 / (logf(ntc_resistance / NTC_R0) / NTC_BETA + 1 / (T_ABSOLUTE_ZERO+25)) - T_ABSOLUTE_ZERO;
    return temperature;
}

void adc4_read(adcs_t *adcs){
    static uint16_t data[5];
    HAL_ADC_Start_DMA(adcs->adc4, (uint32_t *) data, 5);
    HAL_ADC_PollForConversion(adcs->adc4, 10);

    HAL_Delay(10);



    adcs->motor_temp2 = convert_ntc(data[0]);
    adcs->throttleA = (float)data[1] / 4095.0f;
    adcs->throttleB = (float)data[2] / 4095.0f;

    const float VBUS_VOLTS_PER_BIT = 1.0f/7.0f;
    const uint16_t VBUS_OFFSET = 240;

    adcs->vbus = (float)(data[3] - VBUS_OFFSET) * VBUS_VOLTS_PER_BIT;

    const float INPUT_12V_VOLTS_PER_BIT = VREF / 4095.f * 11;
    adcs->input12V = (float)data[4] * INPUT_12V_VOLTS_PER_BIT;
}

void adc2_read(adcs_t * result)
{
    HAL_ADC_Start(result->adc2);
    HAL_ADC_PollForConversion(result->adc2,10);
    result->transistor1 = convert_ntc(HAL_ADC_GetValue(result->adc2));
}