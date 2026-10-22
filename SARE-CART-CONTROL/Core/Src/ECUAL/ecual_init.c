/******************* Section 0 : Includes *******************/

#include "../../Inc/ECUAL/ecual_init.h"

/******************* Section 1 : Variables Definitions *******************/

/* GPS Channel Pin & Hardware Mapping */
#define ECUAL_GPS_UART_ID        (SIM7600_UART_1)
#define ECUAL_GPS_TX_PIN         (SIM7600_PIN_PA9)
#define ECUAL_GPS_RX_PIN         (SIM7600_PIN_PA10)
#define ECUAL_GPS_BAUD           (9600UL)       /* Temporarily set to 9600 baud */
#define ECUAL_GPS_RX_BUF_BYTES   (1024U)
#define ECUAL_GPS_NVIC_PRI       (8U)

/* GPS RX Ring Buffer (storage for port IRQ driver) */
static uint8 m_gps_rx_buf[ECUAL_GPS_RX_BUF_BYTES];

/* Port Configuration for SIM7600 GPS */
static sim7600_port_config_t m_gps_port_cfg = {
    .uart_id        = ECUAL_GPS_UART_ID,
    .tx_pin         = ECUAL_GPS_TX_PIN,
    .rx_pin         = ECUAL_GPS_RX_PIN,
    .baud_rate      = ECUAL_GPS_BAUD,
    .word_len       = SIM7600_WORD_LEN_8,
    .parity         = SIM7600_PARITY_NONE,
    .stop_bits      = SIM7600_STOP_BITS_1,
    .oversampling   = SIM7600_OVERSAMPLE_16,
    .flow_control   = SIM7600_FLOW_NONE,
    .cts_pin        = SIM7600_PIN_NONE,
    .rts_pin        = SIM7600_PIN_NONE,
    .rx_buffer      = &m_gps_rx_buf[0],
    .rx_buffer_size = (uint16)ECUAL_GPS_RX_BUF_BYTES,
    .nvic_priority  = ECUAL_GPS_NVIC_PRI
};

/* SIM7600 GPS Layer Configuration */
static sim7600_gps_cfg_t m_gps_cfg = {
    .port_cfg        = &m_gps_port_cfg,
    .auto_power_on   = true,
    .checksum_strict = true
};

/************** Section 2 : Helper Functions Declarations **************/

/*************** Section 3 : Software Interfaces Definitions (APIs) ****************/

Std_ReturnType ecual_init(void) {
    Std_ReturnType Retval = E_OK;

    /* Initialize SIM7600 GPS Module */
    Retval = sim7600_gps_init(&m_gps_cfg);

    return Retval;
}

/************** Section 4 : Helper Functions Definitions **************/
