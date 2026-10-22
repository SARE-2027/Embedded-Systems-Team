/******************* Section 0 : File Guard *******************/

#ifndef __SIM7600_GPS_DEBUG_H
#define __SIM7600_GPS_DEBUG_H

/*
 * ==================================================================
 *  !!!  TEST-ONLY MODULE - PHASE 3 (delete me when done testing) !!!
 * ==================================================================
 *
 * Streams, over a dedicated debug UART, everything the SIM7600E-H
 * GPS module sends:
 *    - Every raw line as it arrives (NMEA + AT informational lines).
 *    - A formatted parsed-FIX summary once every SUMMARY period.
 *
 * Wiring assumed (this phase's test setup):
 *    SIM7600E-H  ->  USART1 (PA9 = TX,  PA10 = RX)   GPS channel
 *    Debug dump  ->  USART2 (PA2 = TX,  PA3  = RX)   debug OUT channel
 *
 * All channels / pins / baud / periods are cfg macros below (Section 2),
 * so the test can be re-pointed at any factory-mapped USART during bring-up.
 *
 * This module is NOT part of the permanent driver stack:
 *    - do NOT include it from ecual_init
 *    - it owns its own kernel (sim7600_port + sim7600_gps) instances,
 *      its own ring buffers, and prints with tiny fixed-point formatters
 *      (no printf, no floats).
 */

/******************* Section 1 : Includes *******************/

#include "../../MCAL/mcal_std_types.h"

#include "sim7600_port.h"

/******************* Section 2 : Macros Declarations *******************/

/* --- GPS channel (where the SIM7600E-H module is wired) --- */
#define SIM7600_GPS_DEBUG_UART_GPS          (SIM7600_UART_1)
#define SIM7600_GPS_DEBUG_TX_PIN_GPS        (SIM7600_PIN_PA9)
#define SIM7600_GPS_DEBUG_RX_PIN_GPS        (SIM7600_PIN_PA10)
#define SIM7600_GPS_DEBUG_NVIC_PRI_GPS      (8U)

/* --- Debug dump channel (raw NMEA + parsed data OUT terminal) --- */
#define SIM7600_GPS_DEBUG_UART_DBG          (SIM7600_UART_2)
#define SIM7600_GPS_DEBUG_TX_PIN_DBG        (SIM7600_PIN_PA2)
#define SIM7600_GPS_DEBUG_RX_PIN_DBG        (SIM7600_PIN_PA3)   /* never used (TX-only test) */
#define SIM7600_GPS_DEBUG_NVIC_PRI_DBG      (12U)

/* --- Both channels run the SIM7600 default framing --- */
#define SIM7600_GPS_DEBUG_BAUD              (115200UL)

/* --- Ring buffer sizes --- */
#define SIM7600_GPS_DEBUG_GPS_RX_BUF_BYTES  (1024U)    /* NMEA stream          */
#define SIM7600_GPS_DEBUG_DBG_RX_BUF_BYTES  (32U)      /* TX-only channel      */

/* --- Parsed-data summary period (ms) --- */
#define SIM7600_GPS_DEBUG_SUMMARY_MS        (1000UL)

/******************* Section 3 : Macros Functions Declarations **************/

/* (none) */

/************** Section 4 : User Defined Data Types & Variables Declarations
 * **************/

/* (none - test module, no public types) */

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/*
 * Brings up BOTH instances of the sim7600_port core (debug UART first, so
 * boot progress can be reported), then initializes the GPS layer with the
 * auto GNSS boot sequence (ATE0 + CGNSSPWR=1 + CGNSSTST=1).
 *
 * Blocks ~2-16 s during the AT exchanges on first boot - expected in test.
 *
 * NOTE - run AFTER HAL_Init() + SystemClock_Config() (needs HAL tick + it
 * powers the GPIO/USART clocks itself).
 */
Std_ReturnType sim7600_gps_debug_init(void);

/*
 * Periodic service - call from the main while(1) loop, never blocks on data.
 *   - prints every completed raw line as it arrives   ("RAW | ...")
 *   - prints the parsed FIX snapshot every SUMMARY_MS ("FIX/POS/ALT/SPD/...")
 */
void sim7600_gps_debug_task(void);

#endif /* __SIM7600_GPS_DEBUG_H */
