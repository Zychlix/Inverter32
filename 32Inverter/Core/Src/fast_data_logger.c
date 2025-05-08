#include "fast_data_logger.h"


FDL_RETURN_VALUE fdl_init(fdl_t * instance)
{
    if(!instance) return -1;

    instance->input_data_index = 0; //Data starts from 0
    instance->armed = false;


    return 0;
}

FDL_RETURN_VALUE fdl_arm(fdl_t * instance)
{
    if(!instance) return -1;

    instance->armed = true;

    return 0;
}

FDL_RETURN_VALUE fdl_add_datapoint(fdl_t * instance, fdl_data_t * data )
{
    if(!instance) return -1;
if(instance->armed) {
    if (instance->input_data_index < FDL_POINT_COUNT) {
        instance->data[instance->input_data_index].x = data->x;
        instance->data[instance->input_data_index].y = data->y;

        instance->input_data_index += 1;
    } else {
        instance->armed = false;
        fdl_acquisition_complete();
    }
}


    return 0;

}

FDL_RETURN_VALUE fdl_return_dataset(fdl_t * instance);  //Can be point by point, but whatever

