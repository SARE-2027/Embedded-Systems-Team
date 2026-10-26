/******************* Section 0 : File Guard *******************/

#ifndef __ECUAL_INIT_H
#define __ECUAL_INIT_H

/******************* Section 1 : Includes *******************/

#include "../MCAL/mcal_std_types.h"
#include "SIM7600EH/sim7600_port.h"
#include "SIM7600EH/sim7600_gps.h"
#include "Hx711/hx711.h"

/******************* Section 2 : Macros Declarations *******************/


/************** Section 3 : Macros Functions Declarations **************/



/************** Section 4 : User Defined Data Types & Variables Declarations **************/



/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

Std_ReturnType ecual_init(void);

/* Cooperative service for all initialized ECUAL components. Call frequently in main loop. */
void ecual_task(void);

/* Application-facing data/control APIs; component handles remain private to ECUAL. */
bool ecual_gps_is_ready(void);
bool ecual_hx711_is_ready(void);
Std_ReturnType ecual_gps_get_data(sim7600_gps_data_t *out);
Std_ReturnType ecual_hx711_tare(uint8 samples);
bool ecual_hx711_get_weight_mg(int32 *weight_mg);

#endif /* __ECUAL_INIT_H */
