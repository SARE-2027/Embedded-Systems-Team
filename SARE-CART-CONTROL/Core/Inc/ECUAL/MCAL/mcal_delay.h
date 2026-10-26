/******************* Section 0 : File Guard *******************/

#ifndef MCAL_DELAY_H
#define MCAL_DELAY_H

/******************* Section 1 : Includes *******************/

#include "mcal_std_types.h"

/******************* Section 2 : Macros Declarations *******************/

/************** Section 3 : Macros Functions Declarations **************/

/************** Section 4 : User Defined Data Types & Variables Declarations **************/

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/** Blocking delay. Needs the SysTick tick running (after HAL_Init); do not call from an ISR
 *  whose priority is not higher than SysTick's. */
void     Mcal_Delay_Ms(uint32_t ms);

/** Milliseconds since start-up (wraps after ~49 days; use unsigned subtraction for elapsed time). */
uint32_t Mcal_GetTick_Ms(void);

#endif /* MCAL_DELAY_H */
