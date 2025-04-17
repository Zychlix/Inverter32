#pragma once
#include "stdint.h"

#define DEZHOU_STATUS_EXTID (0x18FF50E5)
#define DEZHOU_COMMAND_EXTID (0x1806E5F4)

typedef struct __attribute__((__packed__))
{
    uint16_t voltage_msb;
    uint16_t current_msb;
    uint8_t  hardware_failure:1;
    uint8_t  over_temperature:1;
    uint8_t  ac_error:1;
    uint8_t  battery_error:1;
    uint8_t  communication_timeout:1;

    uint8_t  _x0:3;
    uint8_t _x1[3];

} dezhou_status_frame_t;


typedef struct
{
    uint16_t voltage_setpoint_msb;
    uint16_t current_setpoint_msb;
    uint8_t battery_contactor;
    uint8_t mode;
    uint8_t _x0[2];
} dezhou_command_frame_t;