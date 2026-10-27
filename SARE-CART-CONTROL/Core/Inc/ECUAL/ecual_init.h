/******************* Section 0 : File Guard *******************/

#ifndef __ECUAL_INIT_H
#define __ECUAL_INIT_H

/******************* Section 1 : Includes *******************/

#include "../MCAL/mcal_std_types.h"
#include "SIM7600EH/sim7600_port.h"
#include "SIM7600EH/sim7600_gps.h"
#include "Hx711/hx711.h"
#include "NFC/nfc.h"

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
bool ecual_nfc_is_ready(void);

Std_ReturnType ecual_gps_get_data(sim7600_gps_data_t *out);
Std_ReturnType ecual_hx711_tare(uint8 samples);
bool ecual_hx711_get_weight_mg(int32 *weight_mg);

/* zwdt el goz2 dah*/
Std_ReturnType ecual_hx711_calibrate(uint32 known_weight_g, uint8 samples);   /* known weight on cell  */
Std_ReturnType ecual_hx711_set_scale(int32 counts_per_kg);                    /* restore saved scale   */
Std_ReturnType ecual_hx711_set_offset(int32 offset);                          /* restore saved offset  */
Std_ReturnType ecual_hx711_get_scale(int32 *counts_per_kg);
Std_ReturnType ecual_hx711_get_offset(int32 *offset);
bool           ecual_hx711_get_raw(int32 *raw);
/* zwdt el goz2 dah*/

bool ecual_nfc_get_card(nfc_card_t *out_card);

#endif /* __ECUAL_INIT_H */

