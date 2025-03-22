//
// Created by michal on 17.06.23.
//

#include "adc.h"
#include "math.h"


const float VREF = 3.3f;
const float NTC_V_HIGH = 5.0f;
const float NTC_BETA = 3500; // TODO: CZY MOŻE 3300?
const float NTC_R0_IGBT = 5e3f;
const float NTC_R0_MOTOR = 47e3f;
const float NTC_R_LOWSIDE_IGBT = 680;
const float NTC_R_HIGHSIDE_MOTOR = 1e4f;
const float T_ABSOLUTE_ZERO = 273;

#define INV_TEMP_FILTER_ALPHA 0.01f
#define INV_THROTTLE_FILTER_ALPHA 0.05f
#define INV_VBUS_FILTER_ALPHA 0.01f //0.005


void exp_filter(float * output_value, float input, float alpha)
{
    *output_value = input * alpha + (1-alpha)*(*output_value);
}

float convert_ntc(uint16_t raw, float r0_ntc, float r_lowside, float v_high){
    float adc_voltage = (VREF * (float)raw/ 4095.f);
    float ntc_resistance = r_lowside * (NTC_V_HIGH / adc_voltage - 1);
    float temperature = 1 / (logf(ntc_resistance / r0_ntc) / NTC_BETA + 1 / (T_ABSOLUTE_ZERO+25)) - T_ABSOLUTE_ZERO;
    return temperature;
}

float convert_ntc_highside(uint16_t raw, float r0_ntc, float r_highside, float v_high){
    float adc_voltage = (VREF * (float)raw/ 4095.f);
    float r_lowside = r_highside / (VREF / adc_voltage - 1);
    float temperature = 1 / (logf(r_lowside / r0_ntc) / NTC_BETA + 1 / (T_ABSOLUTE_ZERO+25)) - T_ABSOLUTE_ZERO;
    return temperature;
}


void adc4_read(adcs_t *adcs){
    static uint16_t data[5];
    HAL_ADC_Start_DMA(adcs->adc4, (uint32_t *) data, 5);
    HAL_ADC_PollForConversion(adcs->adc4, 10);

    float motor_temp2_current = convert_ntc_highside(data[0],NTC_R0_MOTOR, NTC_R_HIGHSIDE_MOTOR, 3.3f);
    if(motor_temp2_current>-270)
    {
        exp_filter(&adcs->motor_temp2, motor_temp2_current, 0.01f); //Why doesnt it work???

    }

//    adcs->motor_temp2 = motor_temp2_current;
//    adcs->motor_temp2 = convert_ntc(data[0],NTC_R0_MOTOR, NTC_R_LOWSIDE_MOTOR);

    float throttle_a_current = (float)data[1] / 4095.0f;
    float throttle_b_current = (float)data[2] / 4095.0f;

    exp_filter(&adcs->throttleA, throttle_a_current, INV_THROTTLE_FILTER_ALPHA);
    exp_filter(&adcs->throttleB, throttle_b_current, INV_THROTTLE_FILTER_ALPHA);

    const float VBUS_VOLTS_PER_BIT = 1.0f/7.0f;
    const uint16_t VBUS_OFFSET = 480;

    static volatile float new_vbus;
    new_vbus = (float)((int16_t)data[3] - VBUS_OFFSET) * VBUS_VOLTS_PER_BIT;
    exp_filter(&adcs->vbus, new_vbus,INV_VBUS_FILTER_ALPHA);


    const float INPUT_12V_VOLTS_PER_BIT = VREF / 4095.f * 11;
    adcs->input12V = (float)data[4] * INPUT_12V_VOLTS_PER_BIT;

}

void adc2_read(adcs_t * result)
{
    HAL_ADC_Start(result->adc2);
    HAL_ADC_PollForConversion(result->adc2,10);
    float current_temp_value = convert_ntc(HAL_ADC_GetValue(result->adc2), NTC_R0_IGBT,NTC_R_LOWSIDE_IGBT,VREF);
    exp_filter(&result->transistor1,current_temp_value, INV_TEMP_FILTER_ALPHA);
}