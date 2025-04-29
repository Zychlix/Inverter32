#pragma once
#include "stm32f3xx.h"

#define CDI_RETURN_VALUE int32_t

#define CDI_CAN_STDID 0x100

#define CDI_CHANNEL_T uint8_t
//typedef struct
//{
//
//};
//cdi_channel_t;


typedef struct
{
    CAN_HandleTypeDef * can; //Used instance

    CAN_TxHeaderTypeDef tx_header;



} cdi_t;

CDI_RETURN_VALUE cdi_init(cdi_t * instance);    //Object initialize. Returns 0 on success

CDI_RETURN_VALUE cdi_transmit_channel(cdi_t * instance, CDI_CHANNEL_T channel, uint8_t * data, uint8_t size); //Send data to given channel