/******************* Section 0 : File Guard *******************/

#ifndef __MCAL_STD_TYPES_H
#define __MCAL_STD_TYPES_H

/******************* Section 1 : Includes *******************/

#include "mcal_std_lib.h"

/******************* Section 2 : Macros Declarations *******************/



/************** Section 3 : Macros Functions Declarations **************/

#define set_bit(Reg, Pos) ((Reg) |= (1U << (Pos)))
#define clr_bit(Reg, Pos) ((Reg) &= ~(1U << (Pos)))
#define tog_bit(Reg, Pos) ((Reg) ^= (1U << (Pos)))
#define read_bit(Reg, Pos) (((Reg) >> (Pos)) & 0x01U)

#define HWREG8(__X) (*((volatile uint8 *)(__X)))
#define HWREG16(__X) (*((volatile uint16 *)(__X)))
#define HWREG32(__X) (*((volatile uint32 *)(__X)))
#define HWREG64(__X) (*((volatile uint64 *)(__X)))

/************** Section 4 : User Defined Data Types & Variables Declarations
 * **************/

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long uint32;
typedef unsigned long long uint64;
typedef char int8;
typedef short int16;
typedef long int32;
typedef long long int64;

/*************** Section 5 : Software Interfaces Declarations (APIs)
 * ***************/

typedef enum { E_NOT_OK = (uint8)0x00, E_OK = (uint8)0x01 } Std_ReturnType;

#endif /* __MCAL_STD_TYPES_H */