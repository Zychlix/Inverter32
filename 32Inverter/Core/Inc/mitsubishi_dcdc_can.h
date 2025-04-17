#pragma once
#include "stdint.h"

typedef struct {
    int16_t A0;           //-40 + 1*temp
    int16_t A1;
    int16_t A2;
    int16_t B0;
    int16_t B1;
}DCDC_Converted_Data_Temperatures;

typedef struct{

    float voltage;    //0.01V/LSB
    float current;           //0.1A/lSb
    bool charging_active;

}DCDC_Converted_Data_Aux_Battery;

typedef struct {


    float voltage;
    float current;
    bool  hardware_failure;
    bool  over_temperature;
    bool  ac_error;
    bool  battery_error;
    bool  communication_timeout;


    bool new_frame;         //Set on every write


} DCDC_Converted_Data_t;


typedef struct __attribute__((__packed__))
{
    uint16_t charge_voltage;
    uint8_t charge_current;
    uint8_t _nn0[5];
}DCDC_Frame_Setpoint_x286_t; //286


typedef struct __attribute__((__packed__))
{
    uint8_t _nn[2];
    uint8_t enable_0xb6; //B6 enables charging
    uint8_t _nn1[5];
}DCDC_Frame_Enable_x285_t; //285 Should be sent every 100ms


typedef struct __attribute__((__packed__))
{
    uint16_t aux_battery_voltage;    //0.01V/LSB
    uint16_t aux_current;           //0.1A/lSb
    uint8_t temperature_1;           //-40 + 1*temp
    uint8_t temperature_2;
    uint8_t temperature_3;
    unsigned int error : 1 ;
    unsigned int in_operation: 1 ;
    unsigned int _nn1 : 1;
    unsigned int _nn2 : 1 ;
    unsigned int ready :1;
    unsigned int _nn3 : 1;
    unsigned int _nn4 :2;

}DCDC_Frame_Status_377_t;


typedef struct __attribute__((__packed__))
{
    uint8_t battery_voltage;    //*2
    uint8_t supply_voltage;     // Volts
    uint8_t charge_current;     // Volts
    uint8_t temperature_1;     // Volts
    uint8_t temperature_2;     // Volts
    unsigned int _nb1:1;
    unsigned int mains_present:1;
    unsigned int _nb2:1;
    unsigned int charging:1;
    unsigned int error_can:1;
    unsigned int _nb3:1;
    unsigned int dcdc_active:1;
    unsigned int pilot_present:1;
    uint8_t ac_current;      //x10?
    uint8_t dc_current;      //x10? Should equal setpoint

}DCDC_Frame_Main_Battery_389_t;


typedef struct __attribute__((__packed__))
{
    uint8_t temp_0;    //*2
    uint8_t temp_1;    //*2
    uint8_t DC_voltage;    //*2
    uint8_t evse_duty;
    unsigned int _nn0:2;
    unsigned int waiting_for_mains:1;
    unsigned int ready_for_charging:1;
    unsigned int _nn1:4;
    uint8_t _byte_0;
    uint8_t _byte_1;
    uint8_t _byte_2;


}DCDC_Frame_Main_Battery_38A_t;

typedef struct __attribute__((__packed__))
{
    bool frame_377_received;    //Set on receive. Reset on parse
    bool frame_389_received;
    bool frame_38A_received;

    DCDC_Frame_Status_377_t status;
    DCDC_Frame_Main_Battery_389_t main_battery_status;
    DCDC_Frame_Main_Battery_38A_t evse;
} DCDC_Charger_t;

/*
 * 0x285 is sent every 100 ms
 */