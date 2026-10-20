/******************* Section 0 : File Guard *******************/

#ifndef __SIM7600_PORT_CFG_H
#define __SIM7600_PORT_CFG_H

/******************* Section 1 : Includes *******************/

/* (empty - pure configuration header) */

/******************* Section 2 : Macros Declarations *******************/

/*
 * ===========================================================================
 *  sim7600_port DEFAULT INSTANCE CONFIGURATION
 * ===========================================================================
 *  Edit the macros below OR build your own sim7600_port_config_t at runtime.
 *  Both paths are fully supported; the default config below simply removes
 *  boilerplate from the application for the single-instance case.
 *
 *  +--------+------------------+-----------------+------------------------+
 *  | USART  | TX Alternatives  | RX Alternatives | AF | APB Bus            |
 *  +--------+------------------+-----------------+------------------------+
 *  | USART1 | PA9, PB6         | PA10, PB7       | 7  | APB2 (max 84 MHz)  |
 *  | USART2 | PA2              | PA3             | 7  | APB1 (max 42 MHz)  |
 *  | USART6 | PA11, PC6        | PA12, PC7       | 8  | APB2 (max 84 MHz)  |
 *  +--------+------------------+-----------------+------------------------+
 *
 *  (Checked against DS9716 Rev 11 - Tables 8/9 + pin definitions.
 *   PD5/PD6 for USART2 exist only on LQFP100-sized parts, NOT on F401RC LQFP64,
 *   so they are intentionally absent from the mapping table.)
 */

/* --- Which physical USART the SIM7600 sits on --- */
#define SIM7600_PORT_UART_ID_DEFAULT SIM7600_UART_2

/* --- Pin Mapping (defaults = UART2 on PA2/PA3) --- */
#define SIM7600_PORT_TX_PIN_DEFAULT SIM7600_PIN_PA2
#define SIM7600_PORT_RX_PIN_DEFAULT SIM7600_PIN_PA3

/* --- Frame Format (SIM7600E-H power-on defaults = 115200 8N1, no HW flow) ---
 */
#define SIM7600_PORT_BAUD_DEFAULT (115200UL)
#define SIM7600_PORT_WORD_LEN_DEFAULT SIM7600_WORD_LEN_8
#define SIM7600_PORT_PARITY_DEFAULT SIM7600_PARITY_NONE
#define SIM7600_PORT_STOP_BITS_DEFAULT SIM7600_STOP_BITS_1
#define SIM7600_PORT_OVERSAMPLE_DEFAULT SIM7600_OVERSAMPLE_16
#define SIM7600_PORT_FLOW_DEFAULT SIM7600_FLOW_NONE

/* --- RX Ring Buffer (caller-supplied storage, lives in the app/global space)
 * --- */
#define SIM7600_PORT_RX_BUFFER_SIZE_DEFAULT (512U)

/* --- NVIC Preemption Priority (lower number = higher urgency, 0..15) --- */
#define SIM7600_PORT_NVIC_PRIORITY_DEFAULT (5U)

/* --- TX Timeouts --- */
#define SIM7600_PORT_CFG_TX_TIMEOUT_MS (100UL) /* Per send() call deadline  */
#define SIM7600_PORT_CFG_RX_TIMEOUT_MS (100UL) /* Per recv() call deadline  */

/* --- Flow Control Pins (leave SIM7600_PIN_NONE when flow = NONE) --- */
#define SIM7600_PORT_CTS_PIN_DEFAULT SIM7600_PIN_NONE
#define SIM7600_PORT_RTS_PIN_DEFAULT SIM7600_PIN_NONE

/******************* Section 3 : Macros Functions Declarations **************/

/* (none) */

/************** Section 4 : User Defined Data Types & Variables Declarations
 * **************/

/*
 * Default pre-built configuration - ready to pass straight into
 * sim7600_port_init(&sim7600_port_default_config).
 * (Actual declaration lives in sim7600_port.h, once sim7600_port_config_t is
 *  fully defined.)
 */

/*************** Section 5 : Software Interfaces Declarations (APIs)
 * ***************/

#endif /* __SIM7600_PORT_CFG_H */
