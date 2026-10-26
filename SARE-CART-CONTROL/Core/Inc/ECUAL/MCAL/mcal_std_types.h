/******************* Section 0 : File Guard *******************/

#ifndef MCAL_STD_TYPES_H
#define MCAL_STD_TYPES_H

/******************* Section 1 : Includes *******************/

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/******************* Section 2 : Macros Declarations *******************/

/* --- STM32F401 Flash Sizing Guards (informational only) --- */
#define SOC_LOCAL_FLASH_BYTES (256UL * 1024UL) /* 256KB */
#define SOC_LOCAL_RAM_BYTES   (64UL * 1024UL)  /* 64KB  */

/******************* Section 3 : Macros Functions Declarations **************/

#define set_bit(Reg, Pos)  ((Reg) |= (1U << (Pos)))
#define clr_bit(Reg, Pos)  ((Reg) &= ~(1U << (Pos)))
#define tog_bit(Reg, Pos)  ((Reg) ^= (1U << (Pos)))
#define read_bit(Reg, Pos) (((Reg) >> (Pos)) & 0x01U)

#define HWREG8(__X)  (*((volatile uint8 *)(__X)))
#define HWREG16(__X) (*((volatile uint16 *)(__X)))
#define HWREG32(__X) (*((volatile uint32 *)(__X)))
#define HWREG64(__X) (*((volatile uint64 *)(__X)))

/************** Section 4 : User Defined Data Types & Variables Declarations **************/

typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;
typedef int8_t   int8;
typedef int16_t  int16;
typedef int32_t  int32;
typedef int64_t  int64;

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/* NOTE: E_OK = 1 (not the AUTOSAR value 0). Always compare explicitly: == E_OK / != E_OK. */
typedef enum { E_NOT_OK = (uint8)0x00, E_OK = (uint8)0x01 } Std_ReturnType;

#endif /* MCAL_STD_TYPES_H */
