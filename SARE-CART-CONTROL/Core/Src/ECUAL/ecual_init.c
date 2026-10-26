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

/* HX711 Load Cell Pin & Calibration Mapping */
#define ECUAL_HX711_SCK_PIN      (HX711_PIN(B, 0))   /* any pin: HX711_PIN(<port letter>, <number>) */
#define ECUAL_HX711_DOUT_PIN     (HX711_PIN(B, 1))
#define ECUAL_HX711_GAIN         (HX711_GAIN_128)
#define ECUAL_HX711_SCALE_CPK    (100000)       /* Raw counts per kg - PLACEHOLDER, run hx711_calibrate() */

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

/* HX711 Port Configuration & Instance */
static const hx711_cfg_t m_hx711_cfg = {
    .sck_pin   = ECUAL_HX711_SCK_PIN,
    .dout_pin  = ECUAL_HX711_DOUT_PIN,
    .gain      = ECUAL_HX711_GAIN
};

static hx711_t m_hx711;

/* Track successful initialization before dispatching component tasks/APIs. */
static bool m_gps_initialized = false;
static bool m_hx711_initialized = false;

/************** Section 2 : Helper Functions Declarations **************/

/*************** Section 3 : Software Interfaces Definitions (APIs) ****************/

Std_ReturnType ecual_init(void) {
    Std_ReturnType Retval = E_OK;

    m_gps_initialized = false;
    m_hx711_initialized = false;

    /* Initialize SIM7600 GPS Module */
    if (E_OK == sim7600_gps_init(&m_gps_cfg)) {
        m_gps_initialized = true;
    }
    else {
        Retval = E_NOT_OK;
    }

    /* Initialize HX711 Load Cell ADC */
    if (E_OK == hx711_init(&m_hx711, &m_hx711_cfg)) {
        m_hx711_initialized = true;
        (void)hx711_set_scale(&m_hx711, ECUAL_HX711_SCALE_CPK);
    }
    else {
        Retval = E_NOT_OK;
    }

    return Retval;
}

void ecual_task(void) {
    /* Keep component service routines behind the ECUAL layer boundary. */
    if (m_gps_initialized) {
        sim7600_gps_task();
    }
    if (m_hx711_initialized) {
        hx711_task(&m_hx711);
    }
}

bool ecual_gps_is_ready(void) {
    return m_gps_initialized;
}

bool ecual_hx711_is_ready(void) {
    return m_hx711_initialized;
}

Std_ReturnType ecual_gps_get_data(sim7600_gps_data_t *out) {
    if (!m_gps_initialized || (NULL == out)) {
        return E_NOT_OK;
    }
    return sim7600_gps_get_data(out);
}

Std_ReturnType ecual_hx711_tare(uint8 samples) {
    if (!m_hx711_initialized) {
        return E_NOT_OK;
    }
    return hx711_tare(&m_hx711, samples);
}

bool ecual_hx711_get_weight_mg(int32 *weight_mg) {
    if (!m_hx711_initialized) {
        return false;
    }
    return hx711_get_latest_weight_mg(&m_hx711, weight_mg);
}

/************** Section 4 : Helper Functions Definitions **************/
