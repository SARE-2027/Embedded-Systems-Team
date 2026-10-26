/******************* Section 0 : File Guard *******************/

#ifndef MCAL_GPIO_H
#define MCAL_GPIO_H

/******************* Section 1 : Includes *******************/

#include "mcal_std_types.h"

/******************* Section 2 : Macros Declarations *******************/

/************** Section 3 : Macros Functions Declarations **************/

/************** Section 4 : User Defined Data Types & Variables Declarations **************/

/* Pins are addressed by logical ID. Add new IDs here and in mcal_gpio.c.
 * Pin direction / initial level is configured by CubeMX (MX_GPIO_Init). */
typedef enum {
    MCAL_GPIO_NFC_CS = 0,    /* PB0 - PN532 chip select */
    MCAL_GPIO_COUNT
} Mcal_GpioId_t;

typedef enum {
    MCAL_GPIO_LOW  = 0,
    MCAL_GPIO_HIGH = 1
} Mcal_GpioLevel_t;

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/** Drive an output pin. Returns E_NOT_OK for an invalid id. */
Std_ReturnType Mcal_Gpio_Write(Mcal_GpioId_t id, Mcal_GpioLevel_t level);

#endif /* MCAL_GPIO_H */
