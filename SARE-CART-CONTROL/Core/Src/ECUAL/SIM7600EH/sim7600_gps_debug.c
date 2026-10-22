/******************* Section 0 : File Header *******************/

/**
 *  sim7600_gps_debug.c  -  TEST-ONLY dump utility (Phase 3)
 *
 *  Deletes-with-one-file companion of sim7600_gps.h. Prints, over the debug
 *  UART (PA2/PA3), every line the SIM7600E-H sends on the GPS channel plus
 *  a periodic parsed-FIX snapshot. Fixed-point formatters only.
 */

/******************* Section 1 : Includes *******************/

#include "../../../Inc/ECUAL/SIM7600EH/sim7600_gps_debug.h"

#include "../../../Inc/ECUAL/SIM7600EH/sim7600_gps.h"

#include <string.h>

#include "stm32f4xx_hal.h"

/******************* Section 2 : Macros Declarations *******************/

/* --- Line template widths --- */
#define DBG_DIGITS_U32                10U
#define DBG_POW10_MAX                 6U     /* max fixed scale handled */

/******************* Section 3 : Macros Functions Declarations **************/

/* (none) */

/************** Section 4 : User Defined Data Types & Variables Declarations
 * **************/

/* ------------------------------------------------------------------
 * Ring buffers (caller-owned storage for the port layer)
 * ------------------------------------------------------------------ */
static uint8 m_gps_rx_buf[SIM7600_GPS_DEBUG_GPS_RX_BUF_BYTES];
static uint8 m_dbg_rx_buf[SIM7600_GPS_DEBUG_DBG_RX_BUF_BYTES];

/* ------------------------------------------------------------------
 * Port configs - one instance per physical USART (both live in the
 * sim7600_port core simultaneously; MAX_INSTANCES = 3).
 * ------------------------------------------------------------------ */
static sim7600_port_config_t m_gps_port_cfg = {
    .uart_id        = SIM7600_GPS_DEBUG_UART_GPS,
    .tx_pin         = SIM7600_GPS_DEBUG_TX_PIN_GPS,
    .rx_pin         = SIM7600_GPS_DEBUG_RX_PIN_GPS,
    .baud_rate      = SIM7600_GPS_DEBUG_BAUD,
    .word_len       = SIM7600_WORD_LEN_8,
    .parity         = SIM7600_PARITY_NONE,
    .stop_bits      = SIM7600_STOP_BITS_1,
    .oversampling   = SIM7600_OVERSAMPLE_16,
    .flow_control   = SIM7600_FLOW_NONE,
    .cts_pin        = SIM7600_PIN_NONE,
    .rts_pin        = SIM7600_PIN_NONE,
    .rx_buffer      = &m_gps_rx_buf[0],
    .rx_buffer_size = (uint16)(SIM7600_GPS_DEBUG_GPS_RX_BUF_BYTES),
    .nvic_priority  = SIM7600_GPS_DEBUG_NVIC_PRI_GPS
};

static sim7600_port_config_t m_dbg_port_cfg = {
    .uart_id        = SIM7600_GPS_DEBUG_UART_DBG,
    .tx_pin         = SIM7600_GPS_DEBUG_TX_PIN_DBG,
    .rx_pin         = SIM7600_GPS_DEBUG_RX_PIN_DBG,
    .baud_rate      = SIM7600_GPS_DEBUG_BAUD,
    .word_len       = SIM7600_WORD_LEN_8,
    .parity         = SIM7600_PARITY_NONE,
    .stop_bits      = SIM7600_STOP_BITS_1,
    .oversampling   = SIM7600_OVERSAMPLE_16,
    .flow_control   = SIM7600_FLOW_NONE,
    .cts_pin        = SIM7600_PIN_NONE,
    .rts_pin        = SIM7600_PIN_NONE,
    .rx_buffer      = &m_dbg_rx_buf[0],
    .rx_buffer_size = (uint16)(SIM7600_GPS_DEBUG_DBG_RX_BUF_BYTES),
    .nvic_priority  = SIM7600_GPS_DEBUG_NVIC_PRI_DBG
};

static sim7600_gps_cfg_t m_gps_cfg = {
    .port_cfg        = &m_gps_port_cfg,
    .auto_power_on   = true,
    .checksum_strict = true
};

/* ------------------------------------------------------------------
 * Test-module runtime state
 * ------------------------------------------------------------------ */
static struct {
    bool   init;
    uint32 last_summary_tick;
    char   raw_line[SIM7600_GPS_LINE_MAX_BYTES];
} m_dbg;

/******************* Section 5 : Software Interfaces Declarations (APIs) ***************/

/* (public APIs implemented below) */

/******************* Section 6 : Private Functions (Helpers) ******************/

static const uint32 m_pow10[DBG_POW10_MAX + 1U] =
    { 1UL, 10UL, 100UL, 1000UL, 10000UL, 100000UL, 1000000UL };

/* ------------------------------------------------------------------
 * TX helpers on the debug channel (blocking, port layer underneath)
 * ------------------------------------------------------------------ */
static void m_put_str(const char *s)
{
    (void)sim7600_port_send_ascii(SIM7600_GPS_DEBUG_UART_DBG, s, 100UL);
}

static void m_put_crlf(void)
{
    m_put_str("\r\n");
}

/* Unsigned decimal */
static void m_put_u32(uint32 v)
{
    char   b[DBG_DIGITS_U32 + 1U];
    uint8  i = (uint8)(sizeof(b) - 1U);

    b[i] = '\0';
    do
    {
        i--;
        b[i] = (char)('0' + (v % 10UL));
        v /= 10UL;
    } while (v > 0UL);

    m_put_str(&b[i]);
}

/*
 * Fixed-point print:  val / 10^frac  with zero-padded fraction.
 *   m_put_fixed(-29915123, 6U)  ->  "-29.915123"
 *   m_put_fixed(90, 2U)         ->  "0.90"
 */
static void m_put_fixed(int32 val, uint8 frac)
{
    uint32 mag;
    uint32 div;
    uint32 int_part;
    uint32 fpart;

    if (frac > DBG_POW10_MAX) { frac = DBG_POW10_MAX; }

    if (val < 0)  { m_put_str("-"); mag = (uint32)(-(val + 1)) + 1UL; }
    else          { mag = (uint32)val; }

    div      = m_pow10[frac];
    int_part = mag / div;
    fpart    = mag % div;

    m_put_u32(int_part);
    m_put_str(".");
    {
        /* zero-pad the fraction field before the digits */
        uint32 scale = div / 10UL;
        while (scale > fpart)
        {
            if (scale > 1UL) { m_put_str("0"); }
            scale /= 10UL;
        }
    }
    m_put_u32(fpart);
}

/* ------------------------------------------------------------------
 * One parsed-snapshot block
 * ------------------------------------------------------------------ */
static void m_print_summary(const sim7600_gps_data_t *d)
{
    sim7600_gps_stats_t st;

    m_put_str("t=");            m_put_u32(HAL_GetTick());
    m_put_str(" ---- SIM7600 GPS FIX ----");
    m_put_crlf();

    /* FIX validity */
    m_put_str("FIX | valid=");
    m_put_str(d->valid ? "Y" : "N");
    m_put_str(" q=");
    m_put_u32(d->fix_quality);
    m_put_str(" sats_use=");
    m_put_u32(d->sats_used);
    m_put_str(" sats_vis=");
    m_put_u32(d->sats_visible);
    m_put_str(" hdop=");
    m_put_fixed((int32)d->hdop_cd, 2U);
    m_put_crlf();

    /* Position: absolute value + hemisphere letter */
    m_put_str("POS | lat=");
    m_put_fixed(d->lat_e6, 6U);
    m_put_str((  d->lat_e6 < 0) ? " S" : " N");
    m_put_str(" lon=");
    m_put_fixed(d->lon_e6, 6U);
    m_put_str((  d->lon_e6 < 0) ? " W" : " E");
    m_put_str(" alt=");
    m_put_fixed(d->alt_cm, 2U);
    m_put_str(" m");
    m_put_crlf();

    /* Motion */
    m_put_str("SPD | kn=");
    m_put_fixed(d->speed_cknots, 2U);
    m_put_str(" kmh=");
    m_put_fixed(d->speed_ckmh, 2U);
    m_put_str(" crs=");
    m_put_fixed(d->course_cdeg, 2U);
    m_put_crlf();

    /* UTC */
    m_put_str("UTC | ");
    m_put_str(d->utc_time);
    m_put_str("  ");
    m_put_str(d->utc_date);
    m_put_crlf();

    /* Driver counters */
    if (E_OK == sim7600_gps_get_stats(&st))
    {
        m_put_str("CNT | nmea=");
        m_put_u32(st.nmea_sentences);
        m_put_str(" badck=");
        m_put_u32(st.nmea_bad_checksum);
        m_put_str(" perr=");
        m_put_u32(st.nmea_parse_errors);
        m_put_str(" unk=");
        m_put_u32(st.nmea_unknown);
        m_put_str(" ovf=");
        m_put_u32(st.line_overflows);
        m_put_str(" atok=");
        m_put_u32(st.at_ok_count);
        m_put_str(" aterr=");
        m_put_u32(st.at_error_count);
        m_put_str(" atto=");
        m_put_u32(st.at_timeout_count);
        m_put_crlf();
    }

    m_put_crlf();
}

/******************* Section 7 : Public APIs Implementation *******************/

Std_ReturnType sim7600_gps_debug_init(void)
{
    if (m_dbg.init) { return E_OK; }

    memset(&m_dbg, 0, sizeof(m_dbg));

    /* Debug terminal first - boot progress must be visible */
    if (E_OK != sim7600_port_init(&m_dbg_port_cfg)) { return E_NOT_OK; }

    m_put_str("\r\n=== SIM7600E-H GPS TEST DUMP (Phase 3) ===\r\n");
    m_put_str("=== debug: USART2 PA2/PA3 | gps: USART1 PA9/PA10 ===\r\n");
    m_put_str("=== GNSS boot: ATE0 + CGNSSPWR=1 + CGNSSTST=1 ...\r\n");

    if (E_OK != sim7600_gps_init(&m_gps_cfg))
    {
        m_put_str("!!! GPS layer init FAILED (check wiring / module power)\r\n");
        return E_NOT_OK;
    }

    m_put_str("=== GNSS boot done. Streaming raw + parsed below ===\r\n");

    m_dbg.init             = true;
    m_dbg.last_summary_tick = HAL_GetTick();

    return E_OK;
}

void sim7600_gps_debug_task(void)
{
    if (!m_dbg.init) { return; }

    /* 1) Service the GPS layer (drains USART1 into lines + parsers) */
    sim7600_gps_task();

    /* 2) Forward every completed raw line as it arrives */
    while (E_OK == sim7600_gps_get_raw_line(&m_dbg.raw_line[0],
                                            SIM7600_GPS_LINE_MAX_BYTES))
    {
        m_put_str("RAW |");
        m_put_str(&m_dbg.raw_line[0]);
        m_put_crlf();
    }

    /* 3) Periodic parsed snapshot */
    if ((HAL_GetTick() - m_dbg.last_summary_tick) >=
         SIM7600_GPS_DEBUG_SUMMARY_MS)
    {
        m_dbg.last_summary_tick = HAL_GetTick();

        {
            sim7600_gps_data_t d;
            if (E_OK == sim7600_gps_get_data(&d))
            {
                m_print_summary(&d);
            }
        }
    }
}
