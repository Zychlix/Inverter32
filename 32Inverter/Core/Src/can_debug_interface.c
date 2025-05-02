#include "can_debug_interface.h"

CDI_RETURN_VALUE cdi_init(cdi_t * instance)
{
    if(instance == NULL)
    {
        return -1;
    }

    if(instance->can ==  NULL)
    {
        return -1;
    }

//    uint8_t payload[8] = {0};
    CAN_TxHeaderTypeDef tx_header = {0};

    instance->tx_header.StdId = CDI_CAN_STDID;
    instance->tx_header.ExtId = 0;
    instance->tx_header.IDE = CAN_ID_STD;
    instance->tx_header.DLC = 0;
    instance->tx_header.RTR = CAN_RTR_DATA;





    return 0;
}

CDI_RETURN_VALUE cdi_transmit_channel(cdi_t * instance, CDI_CHANNEL_T channel, uint8_t * data, uint8_t size)
{
    //This has to be fast. No nullptr checking and whatnot

    instance->tx_header.DLC = size;
    uint32_t placeholder_mailbox;

    instance->tx_header.StdId = CDI_CAN_STDID + channel;

    HAL_StatusTypeDef retval;
    retval = HAL_CAN_AddTxMessage(instance->can, &instance->tx_header, (uint8_t *) data, &placeholder_mailbox);

    return retval;


}