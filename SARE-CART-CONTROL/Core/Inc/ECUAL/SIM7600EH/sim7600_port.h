/******************* Section 0 : File Guard *******************/

#ifndef __SIM7600_PORT_H
#define __SIM7600_PORT_H

/******************* Section 1 : Includes *******************/

#include "../../MCAL/mcal_std_types.h"
#include "sim7600_port_cfg.h"

/******************* Section 2 : Macros Declarations *******************/

/* --- Driver Version --- */
#define SIM7600_PORT_VERSION_MAJOR      (1U)
#define SIM7600_PORT_VERSION_MINOR      (0U)
#define SIM7600_PORT_VERSION_PATCH      (0U)

/* --- Maximum Simultaneous UART Instances (one per physical USART) --- */
#define SIM7600_PORT_MAX_INSTANCES      (3U)     /* USART1, USART2, USART6 */

/* --- Timeout Sentinel --- */
#define SIM7600_PORT_NO_TIMEOUT         (0UL)    /* Poll once and return immediately */

/* --- Invalid Pin Marker --- */
#define SIM7600_PIN_NONE                (0xFFU)  /* No pin / disabled feature */

/******************* Section 3 : Macros Functions Declarations **************/

/* --- Convenience Wrapper: Send a NUL-Terminated ASCII String --- */
#define SIM7600_PORT_SEND_STR(id, str) \
    sim7600_port_send_ascii((id), (str), SIM7600_PORT_CFG_TX_TIMEOUT_MS)

/******************* Section 4 : User Defined Data Types & Variables Declarations
 * **************/

/* --- Physical UART Channel Selector --- */
typedef enum {
    SIM7600_UART_1 = 0,      /* USART1 - APB2  */
    SIM7600_UART_2 = 1,      /* USART2 - APB1  */
    SIM7600_UART_6 = 2       /* USART6 - APB2  */
} sim7600_uart_id_t;

/*
 * Physical Pin Selector - Encoded [Port nibble : Pin nibble]
 *   Port: 0x00 = GPIOA, 0x10 = GPIOB, 0x20 = GPIOC, 0x30 = GPIOD
 *
 * NOTE (STM32F401RC - LQFP64):
 *   The driver validates requested pins against the factory mapping table
 *   (sim7600_port.c). Pins that do NOT exist on the LQFP64 package
 *   (e.g. PD5 / PD6 for USART2) are rejected at runtime.
 */
typedef enum {
    /* ---- Port A ---- */
    SIM7600_PIN_PA0  = 0x00U, SIM7600_PIN_PA1  = 0x01U, SIM7600_PIN_PA2  = 0x02U,
    SIM7600_PIN_PA3  = 0x03U, SIM7600_PIN_PA4  = 0x04U, SIM7600_PIN_PA5  = 0x05U,
    SIM7600_PIN_PA6  = 0x06U, SIM7600_PIN_PA7  = 0x07U, SIM7600_PIN_PA8  = 0x08U,
    SIM7600_PIN_PA9  = 0x09U, SIM7600_PIN_PA10 = 0x0AU, SIM7600_PIN_PA11 = 0x0BU,
    SIM7600_PIN_PA12 = 0x0CU, SIM7600_PIN_PA13 = 0x0DU, SIM7600_PIN_PA14 = 0x0EU,
    SIM7600_PIN_PA15 = 0x0FU,
    /* ---- Port B ---- */
    SIM7600_PIN_PB0  = 0x10U, SIM7600_PIN_PB1  = 0x11U, SIM7600_PIN_PB2  = 0x12U,
    SIM7600_PIN_PB3  = 0x13U, SIM7600_PIN_PB4  = 0x14U, SIM7600_PIN_PB5  = 0x15U,
    SIM7600_PIN_PB6  = 0x16U, SIM7600_PIN_PB7  = 0x17U, SIM7600_PIN_PB8  = 0x18U,
    SIM7600_PIN_PB9  = 0x19U, SIM7600_PIN_PB10 = 0x1AU, SIM7600_PIN_PB11 = 0x1BU,
    SIM7600_PIN_PB12 = 0x1CU, SIM7600_PIN_PB13 = 0x1DU, SIM7600_PIN_PB14 = 0x1EU,
    SIM7600_PIN_PB15 = 0x1FU,
    /* ---- Port C ---- */
    SIM7600_PIN_PC0  = 0x20U, SIM7600_PIN_PC1  = 0x21U, SIM7600_PIN_PC2  = 0x22U,
    SIM7600_PIN_PC3  = 0x23U, SIM7600_PIN_PC4  = 0x24U, SIM7600_PIN_PC5  = 0x25U,
    SIM7600_PIN_PC6  = 0x26U, SIM7600_PIN_PC7  = 0x27U, SIM7600_PIN_PC8  = 0x28U,
    SIM7600_PIN_PC9  = 0x29U, SIM7600_PIN_PC10 = 0x2AU, SIM7600_PIN_PC11 = 0x2BU,
    SIM7600_PIN_PC12 = 0x2CU, SIM7600_PIN_PC13 = 0x2DU, SIM7600_PIN_PC14 = 0x2EU,
    SIM7600_PIN_PC15 = 0x2FU,
    /* ---- Port D (not available on LQFP64, kept for portability) ---- */
    SIM7600_PIN_PD0  = 0x30U, SIM7600_PIN_PD1  = 0x31U, SIM7600_PIN_PD2  = 0x32U,
    SIM7600_PIN_PD3  = 0x33U, SIM7600_PIN_PD4  = 0x34U, SIM7600_PIN_PD5  = 0x35U,
    SIM7600_PIN_PD6  = 0x36U, SIM7600_PIN_PD7  = 0x37U, SIM7600_PIN_PD8  = 0x38U,
    SIM7600_PIN_PD9  = 0x39U, SIM7600_PIN_PD10 = 0x3AU, SIM7600_PIN_PD11 = 0x3BU,
    SIM7600_PIN_PD12 = 0x3CU, SIM7600_PIN_PD13 = 0x3DU, SIM7600_PIN_PD14 = 0x3EU,
    SIM7600_PIN_PD15 = 0x3FU
} sim7600_pin_t;

/* --- Parity Selection --- */
typedef enum {
    SIM7600_PARITY_NONE = 0,   /* Default for SIM7600E-H */
    SIM7600_PARITY_EVEN = 1,   /* PCE = 1, PS = 0 */
    SIM7600_PARITY_ODD  = 2    /* PCE = 1, PS = 1 */
} sim7600_parity_t;

/*
 * NOTE (RM0368): the parity bit is carried INSIDE the word-length field.
 * For an "8 data bits + parity" frame (e.g. 8-O-1 / 8-E-1), select
 * SIM7600_WORD_LEN_9 together with a parity value; with WORD_LEN_8 the
 * parity bit consumes the MSB and the payload becomes 7 data bits.
 */

/* --- Stop Bits Selection --- */
typedef enum {
    SIM7600_STOP_BITS_1    = 0,  /* Default for SIM7600E-H */
    SIM7600_STOP_BITS_1_5  = 1,  /* Valid only with 8 data bits? -> see RM0368      */
    SIM7600_STOP_BITS_2    = 2   /* Recommended when parity is enabled              */
} sim7600_stop_bit_t;

/* --- Data Bits Selection --- */
typedef enum {
    SIM7600_WORD_LEN_8 = 0,      /* Default for SIM7600E-H */
    SIM7600_WORD_LEN_9 = 1
} sim7600_word_len_t;

/* --- Oversampling Mode --- */
typedef enum {
    SIM7600_OVERSAMPLE_16 = 0,   /* Default. Best noise immunity                     */
    SIM7600_OVERSAMPLE_8  = 1    /* Up to 2x baud rate (USART1/6 up to 10.5 Mbit/s)  */
} sim7600_oversample_t;

/* --- Hardware Flow Control (USART1 / USART2 only, USART6 = N.A.) --- */
typedef enum {
    SIM7600_FLOW_NONE   = 0,     /* Default for SIM7600E-H (AT+IFC=0,0) */
    SIM7600_FLOW_RTS    = 1,
    SIM7600_FLOW_CTS    = 2,
    SIM7600_FLOW_CTS_RTS = 3
} sim7600_flow_t;

/*
 * Port Layer Configuration - One instance per physical USART.
 *
 * The user supplies the ring-buffer storage (rx_buffer / rx_buffer_size),
 * so buffer placement (e.g. DMA-friendly sections, ECC RAM...) stays 100%
 * under application control. The buffer must outlive the UART session.
 */
typedef struct {
    sim7600_uart_id_t      uart_id;         /* Which USART channel              */
    sim7600_pin_t          tx_pin;          /* TX pin (USART -> SIM7600 RXD)    */
    sim7600_pin_t          rx_pin;          /* RX pin (USART <- SIM7600 TXD)    */
    uint32                 baud_rate;       /* e.g. 115200U (SIM7600 default)   */
    sim7600_word_len_t     word_len;        /* 8 bits typical                   */
    sim7600_parity_t       parity;          /* NONE typical                     */
    sim7600_stop_bit_t     stop_bits;       /* 1 stop bit typical               */
    sim7600_oversample_t   oversampling;    /* 16x typical                      */
    sim7600_flow_t         flow_control;    /* NONE typical (SIM7600 UART1)     */
    sim7600_pin_t          cts_pin;         /* Required if flow uses CTS        */
    sim7600_pin_t          rts_pin;         /* Required if flow uses RTS        */
    uint8                 *rx_buffer;       /* Caller-owned ring buffer storage */
    uint16                 rx_buffer_size;  /* Actual usable size of rx_buffer  */
    uint8                  nvic_priority;   /* 0..15 (preemption, group 4)      */
} sim7600_port_config_t;

/* --- Runtime Transmission/Reception Statistics --- */
typedef struct {
    uint32 rx_bytes;              /* Bytes successfully placed in ring buffer      */
    uint32 tx_bytes;              /* Bytes successfully handed to hardware         */
    uint32 rx_buffer_overflow;    /* Ring buffer full while ISR held new bytes     */
    uint32 hw_overrun;            /* Hardware ORE (RX not serviced fast enough)    */
    uint32 hw_framing_errors;     /* Hardware FE (bad stop bit / baud mismatch)    */
    uint32 hw_noise_errors;       /* Hardware NE (line noise)                      */
    uint32 hw_parity_errors;      /* Hardware PE                                   */
} sim7600_port_stats_t;

/* --- Detailed Status Codes (diagnostics getter) --- */
typedef enum {
    SIM7600_PORT_STATUS_OK           = 0,    /* Last operation succeeded            */
    SIM7600_PORT_STATUS_PARAM        = 1,    /* NULL pointer / zero argument        */
    SIM7600_PORT_STATUS_NOT_INIT     = 2,    /* Instance not initialized            */
    SIM7600_PORT_STATUS_BUSY         = 3,    /* Already initialized / pin in use    */
    SIM7600_PORT_STATUS_INVALID_PIN  = 4,    /* Pin not mapped to this USART (LQFP64) */
    SIM7600_PORT_STATUS_INVALID_BAUD = 5,    /* Baud out of achievable range        */
    SIM7600_PORT_STATUS_TIMEOUT      = 6,    /* TX flag wait / RX data wait timed out */
    SIM7600_PORT_STATUS_FLOW_INVALID = 7,    /* Flow control on USART6 / missing pin */
    SIM7600_PORT_STATUS_HW_FAULT     = 8     /* Framing/Overrun storm detected      */
} sim7600_port_status_t;

/*
 * Default pre-built configuration - built from the macros in
 * sim7600_port_cfg.h (ring buffer storage lives inside sim7600_port.c).
 * Pass it straight to sim7600_port_init(&sim7600_port_default_config).
 */
extern const sim7600_port_config_t sim7600_port_default_config;

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/*
 * NOTE - The SIM7600E-H is connected as a 3-wire UART (TXD/RXD/GND):
 *
 *   STM32F401RCTx                SIM7600E-H
 *   -------------              --------------
 *   TX (e.g. PA2)    ------->  RXD
 *   RX (e.g. PA3)    <-------  TXD
 *   GND              -------   GND
 */

/* --- Lifecycle --- */
Std_ReturnType sim7600_port_init(const sim7600_port_config_t *cfg);
Std_ReturnType sim7600_port_deinit(sim7600_uart_id_t uart_id);
Std_ReturnType sim7600_port_is_init(sim7600_uart_id_t uart_id, bool *is_init);

/* --- TX (Blocking) --- */
Std_ReturnType sim7600_port_send_byte(sim7600_uart_id_t uart_id, uint8 data, uint32 timeout_ms);
Std_ReturnType sim7600_port_send(sim7600_uart_id_t uart_id, const uint8 *data, uint16 len, uint32 timeout_ms);
Std_ReturnType sim7600_port_send_ascii(sim7600_uart_id_t uart_id, const char *str, uint32 timeout_ms);

/* --- RX (Serviced by ISR into caller-owned ring buffer) --- */
Std_ReturnType sim7600_port_recv_byte(sim7600_uart_id_t uart_id, uint8 *data, uint32 timeout_ms);
Std_ReturnType sim7600_port_recv(sim7600_uart_id_t uart_id, uint8 *buf, uint16 max_len,
                                 uint16 *received, uint32 timeout_ms);
Std_ReturnType sim7600_port_rx_available(sim7600_uart_id_t uart_id, uint16 *count);
Std_ReturnType sim7600_port_rx_peek(sim7600_uart_id_t uart_id, uint8 *data, uint16 offset);
Std_ReturnType sim7600_port_rx_flush(sim7600_uart_id_t uart_id);

/* --- Diagnostics --- */
Std_ReturnType sim7600_port_get_stats(sim7600_uart_id_t uart_id, sim7600_port_stats_t *stats);
Std_ReturnType sim7600_port_reset_stats(sim7600_uart_id_t uart_id);
Std_ReturnType sim7600_port_get_last_status(sim7600_uart_id_t uart_id, sim7600_port_status_t *status);

#endif /* __SIM7600_PORT_H */
