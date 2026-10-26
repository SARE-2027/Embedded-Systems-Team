/******************* Section 0 : Includes *******************/

#include "../../Inc/MCAL/mcal_delay.h"

/*************** Section 3 : Software Interfaces Definitions (APIs) ****************/

void Mcal_Delay_Ms(uint32_t ms)
{
    HAL_Delay(ms);
}

uint32_t Mcal_GetTick_Ms(void)
{
    return HAL_GetTick();
}
