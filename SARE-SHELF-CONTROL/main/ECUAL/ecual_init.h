/******************* Section 0 : File Guard *******************/

#ifndef __ECUAL_INIT_H
#define __ECUAL_INIT_H

/******************* Section 1 : Includes *******************/

#include "HX711/ecual_hx711.h"

/******************* Section 2 : Macros Declarations *******************/


/************** Section 3 : Macros Functions Declarations **************/



/************** Section 4 : User Defined Data Types & Variables Declarations **************/


extern hx711_config_t loadcell;

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

Std_ReturnType ecual_init(void);

#endif /* __ECUAL_INIT_H */
