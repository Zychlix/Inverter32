#pragma once
#include "stdint.h"
#include "stdbool.h"
#define FDL_RETURN_VALUE int

typedef struct
{
    float x;
    float y;
}fdl_data_t;

#define FDL_POINT_COUNT 2048

typedef struct
{
    bool armed;
    uint32_t input_data_index;
    fdl_data_t data[FDL_POINT_COUNT];


}fdl_t;

FDL_RETURN_VALUE fdl_init(fdl_t * instance); //Initialize logger

FDL_RETURN_VALUE fdl_arm(fdl_t * instance); // Make logger ready to collect data

FDL_RETURN_VALUE fdl_add_datapoint(fdl_t * instance, fdl_data_t * data ); // Add data to datapoint list

FDL_RETURN_VALUE fdl_return_dataset(fdl_t * instance);  //Can be point by point, but whatever

void fdl_acquisition_complete();