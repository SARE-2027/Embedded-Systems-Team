/******************* Section 0 : File Guard *******************/

#ifndef MCAL_SPI_H
#define MCAL_SPI_H

/******************* Section 1 : Includes *******************/

#include "mcal_std_types.h"

/******************* Section 2 : Macros Declarations *******************/

/************** Section 3 : Macros Functions Declarations **************/

/************** Section 4 : User Defined Data Types & Variables Declarations **************/

typedef enum {
    MCAL_SPI_1 = 0,           /* hspi1 (CubeMX) */
    MCAL_SPI_COUNT
} Mcal_SpiId_t;

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/** Blocking transmit. Returns E_NOT_OK on bad argument, timeout or bus error. */
Std_ReturnType Mcal_Spi_Transmit(Mcal_SpiId_t id, const uint8_t *data,
                                 uint16_t len, uint32_t timeout_ms);

/** Blocking receive (dummy bytes are clocked out). Returns E_NOT_OK on error. */
Std_ReturnType Mcal_Spi_Receive(Mcal_SpiId_t id, uint8_t *data,
                                uint16_t len, uint32_t timeout_ms);

#endif /* MCAL_SPI_H */
