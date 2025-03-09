//
// Created by zychlix on 09.03.25.
//

#include "swo_scope.h"

#include <stm32f303xc.h>
#include <stdint.h>

void swo_send_float(int port, float value)
{
#ifdef DEBUG
    // int32_t q1616 = value * (1 << 16);
    while (ITM->PORT[port].u32 == 0UL)
        {
            __NOP();
        }
    ITM->PORT[port].u32 = *(uint32_t*)&value;
#endif
}
