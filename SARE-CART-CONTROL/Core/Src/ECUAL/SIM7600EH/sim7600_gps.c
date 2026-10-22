/******************* Section 0 : File Header *******************/

/**
 *  sim7600_gps.c  -  SIM7600E-H GNSS Layer (STM32F401RCTx)
 *
 *  Sits on top of the sim7600_port UART core and provides:
 *    - AT command engine (send / collect response / OK-ERROR matching).
 *    - GNSS power control (config-driven boot sequence).
 *    - Streaming NMEA line assembler + checksum validator.
 *    - Fixed-point NMEA parsers: GGA, RMC, GLL, VTG, GSV (talker-agnostic).
 *    - Raw last-line capture (feeds the Phase-3 debug dump).
 *
 *  All fixed point: no floating point (micro-degrees, centi-, deci- scales).
 *  Single-context: task()/send_at()/wait_for_fix() must run in one thread.
 */

/******************* Section 1 : Includes *******************/

#include "../../../Inc/ECUAL/SIM7600EH/sim7600_gps.h"

#include <string.h>

#include "stm32f4xx_hal.h"

/******************* Section 2 : Macros Declarations *******************/

/* --- NMEA sentence codes (matched at line[3]) --- */
#define NMEA_TYPE_GGA              0U
#define NMEA_TYPE_RMC              1U
#define NMEA_TYPE_GLL              2U
#define NMEA_TYPE_VTG              3U
#define NMEA_TYPE_GSV              4U
#define NMEA_TYPE_UNKNOWN          0xFFU

/* --- ASCII helpers --- */
#define ASCII_CR                   ((uint8)0x0D)
#define ASCII_LF                   ((uint8)0x0A)
#define ASCII_DOLLAR               ((uint8)0x24)
#define ASCII_ASTERISK             ((uint8)0x2A)
#define ASCII_COMMA                ((uint8)0x2C)

/* --- Signatures for status lines --- */
#define STATUS_TOKEN_OK            "OK"
#define STATUS_TOKEN_ERROR         "ERROR"
#define STATUS_TOKEN_CME_ERROR     "+CME ERROR"
#define STATUS_TOKEN_CMS_ERROR     "+CMS ERROR"

/* --- Direction markers --- */
#define NMEA_NORTH                 'N'
#define NMEA_SOUTH                 'S'
#define NMEA_EAST                  'E'
#define NMEA_WEST                  'W'

/******************* Section 3 : Macros Functions Declarations **************/

/* (none - GPS_MAX removed: was unused) */

/************** Section 4 : User Defined Data Types & Variables Declarations
 * **************/

/* --- AT exchange state --- */
typedef enum {
    GPS_AT_IDLE = 0,      /* No exchange in progress          */
    GPS_AT_WAIT,          /* Awaiting terminal response       */
    GPS_AT_DONE_OK,
    GPS_AT_DONE_FAIL
} gps_at_state_t;

/* --- Per-layer runtime state (single SIM7600 module) --- */
typedef struct {
    bool                 init;
    sim7600_port_config_t port_copy;      /* Private copy: lifetime-safe */
    bool                 checksum_strict;
    bool                 echo_off_done;

    /* NMEA / AT line assembler */
    char                 line[SIM7600_GPS_LINE_MAX_BYTES];
    uint16               line_len;
    bool                 discarding;      /* Over-length line drain mode */

    /* AT machinery */
    volatile gps_at_state_t at_state;
    char                *at_resp;
    uint16               at_resp_size;
    uint16               at_resp_len;

    /* Latest fix snapshot */
    sim7600_gps_data_t   data;

    /* Raw capture (last completed line, any type) */
    char                 last_line[SIM7600_GPS_LINE_MAX_BYTES];
    bool                 last_line_fresh;

    /* Counters */
    sim7600_gps_stats_t  stats;
} gps_state_t;

static gps_state_t m_gps;

/* --- Default configuration: default port cfg (see sim7600_port_cfg.h) --- */
const sim7600_gps_cfg_t sim7600_gps_default_cfg = {
    .port_cfg        = (sim7600_port_config_t *)&sim7600_port_default_config,
    .auto_power_on   = true,
    .checksum_strict = true
};

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/* (public APIs implemented below) */

/******************* Section 6 : Private Functions (Helpers) *******************/

/* ---------------------------------------------------------------------------
 * Hex nibble/copy utilities
 * ------------------------------------------------------------------------- */
static bool m_hex_to_uint8(const char c, uint8 *out)
{
    if ((c >= '0') && (c <= '9'))      { *out = (uint8)(c - '0');      return true; }
    if ((c >= 'a') && (c <= 'f'))      { *out = (uint8)(c - 'a' + 10); return true; }
    if ((c >= 'A') && (c <= 'F'))      { *out = (uint8)(c - 'A' + 10); return true; }
    return false;
}

/* ---------------------------------------------------------------------------
 * Comma-field splitter for NMEA.
 *   Header ("$GPGGA,") is skipped, so idx 0 is the first field AFTER the
 *   sentence-type token. Returns NULL when the sentence ends before idx.
 *   Empties are valid: returns pointer + len 0.
 * ------------------------------------------------------------------------- */
static const char *m_nmea_field(const char *line, uint8 idx, uint8 *len)
{
    const char *p = line;
    uint8        cur = 0U;
    const char *start = NULL;

    /* Jump past "$XXyyy," (header) */
    while ((*p != '\0') && (*p != ASCII_COMMA)) { p++; }
    if (*p == '\0') { return NULL; }
    p++;                                       /* First field starts here */

    while (cur <= idx)
    {
        start = p;
        while ((*p != '\0') && (*p != ASCII_COMMA)) { p++; }
        if (cur == idx) { break; }
        if (*p == '\0') { return NULL; }
        p++;
        cur++;
    }

    *len = (uint8)(p - start);
    if (start[0] == ASCII_ASTERISK) { return NULL; }  /* Reached checksum tail */
    return start;
}

/* ---------------------------------------------------------------------------
 * Signed decimal parser with fixed scale.
 *   "146.7"  with scale 2 ->  14670
 *   "5150.1234" with scale 4 -> 51501234
 *   ".5"  with scale 2 -> 50
 *   Missing frac digits are zero-padded; extra frac digits are truncated.
 * ------------------------------------------------------------------------- */
static bool m_dec_scaled(const char *s, uint8 len, uint8 scale, int32 *out)
{
    int64  acc   = 0;
    bool   neg   = false;
    bool   seen_digit = false;
    uint8  i     = 0U;
    uint8  frac  = 0U;
    bool   in_frac = false;

    if ((NULL == s) || (0U == len)) { return false; }

    if ('-' == s[0]) { neg = true; i = 1U; }
    else if ('+' == s[0]) { i = 1U; }

    for (; i < len; i++)
    {
        const char c = s[i];

        if (ASCII_ASTERISK == (uint8)c) { break; }        /* Checksum marks end */

        if ((c >= '0') && (c <= '9'))
        {
            seen_digit = true;
            if (!in_frac)
            {
                acc = (acc * 10) + (int64)(c - '0');
                if (acc > ((int64)1 << 40)) { return false; }  /* Guard runaway */
            }
            else if (frac < scale)
            {
                acc = (acc * 10) + (int64)(c - '0');
                frac++;
            }
            /* Extra frac digits beyond scale are ignored (truncation) */
        }
        else if ('.' == c)
        {
            in_frac = true;
        }
        else
        {
            break;      /* Any other character terminates the number  */
        }
    }

    if (!seen_digit) { return false; }

    while (frac < scale)   /* Zero-pad the fraction */
    {
        acc *= 10;
        frac++;
    }

    *out = neg ? (int32)(-acc) : (int32)acc;
    return true;
}

/* ---------------------------------------------------------------------------
 * Unsigned decimal parser (8/16-bit friendly)
 * ------------------------------------------------------------------------- */
static bool m_dec_uint16(const char *s, uint8 len, uint16 *out)
{
    uint32 acc = 0U;
    uint8  i;

    if ((NULL == s) || (0U == len)) { return false; }

    for (i = 0U; i < len; i++)
    {
        const char c = s[i];
        if ((c < '0') || (c > '9')) { return false; }
        acc = (acc * 10U) + (uint32)(c - '0');
        if (acc > 0xFFFFUL) { return false; }
    }

    *out = (uint16)acc;
    return true;
}

/* ---------------------------------------------------------------------------
 * Latitude / longitude decode.
 *   NMEA: lat = "ddmm.mmmm" (2 deg digits), lon = "dddmm.mmmm" (3 deg digits)
 *   deg_e6 = D * 1e6 + round(minutes * 1e6 / 60)
 *   minutes scaled 4 (mmmm) = mm * 10^4
 *     minutes_e6 = (mm_scaled4 * 10^6) / (60 * 10^4) = mm_scaled4 * 10 / 6
 * ------------------------------------------------------------------------- */
static bool m_parse_latlon(const char *f, uint8 len, uint8 deg_digits,
                           char dir, int32 *out_e6)
{
    int32  deg        = 0;
    int32  mm_scaled4 = 0;
    int32  minutes_e6;
    const char *dot   = NULL;
    uint8  int_digits = 0U;
    uint8  i;

    if (((len < (uint8)(deg_digits + 3U))) ||          /* "ddmm.mm" minimums   */
        ((NMEA_NORTH != dir) && (NMEA_SOUTH != dir) &&
         (NMEA_EAST  != dir) && (NMEA_WEST   != dir)))
    {
        return false;
    }

    /* Locate the decimal point; count integer digits before it */
    for (i = 0U; i < len; i++)
    {
        if ('.' == f[i]) { dot = &f[i]; break; }
    }
    int_digits = (NULL == dot) ? len : (uint8)(dot - f);

    if (int_digits != deg_digits)
    {
        return false;    /* dd is always 2, ddd is always 3   */
    }

    if ('.' == f[deg_digits])   /* normative format */
    {
        const uint8 frac_len = (uint8)(len - deg_digits - 1U);
        if (0U == frac_len) { return false; }
        /* Parse "mmmm" scaled by 10^(frac in string), then normalize to 1e4 */
        {
            int32 scaled = 0;
            if (!m_dec_scaled(&f[deg_digits + 1], frac_len, 4U, &scaled))
            {
                return false;
            }
            mm_scaled4 = scaled;
        }
    }
    else    /* No dot: "ddmmyyyy" style means minutes are 2-digit integer    */
    {
        if (!m_dec_scaled(&f[deg_digits], (uint8)(len - deg_digits), 4U, &mm_scaled4))
        {
            return false;
        }
    }

    /* Integer degrees part */
    if (!m_dec_scaled(f, deg_digits, 0U, &deg))
    {
        return false;
    }

    minutes_e6 = ((mm_scaled4 * 10) + 3) / 6;    /* Rounded to micro-degrees */

    *out_e6 = (int32)((uint32)deg * 1000000UL);
    *out_e6 += minutes_e6;

    if ((NMEA_SOUTH == dir) || (NMEA_WEST == dir))
    {
        *out_e6 = -(*out_e6);
    }

    return true;
}

/* ---------------------------------------------------------------------------
 * UTC time "hhmmss.ss" -> "hh:mm:ss"
 * ------------------------------------------------------------------------- */
static void m_parse_time(const char *f, uint8 len, char *dst, uint8 dst_size)
{
    if ((dst_size < 9U) || (len < 6U)) { return; }

    dst[0] = f[0];  dst[1] = f[1];  dst[2] = ':';
    dst[3] = f[2];  dst[4] = f[3];  dst[5] = ':';
    dst[6] = f[4];  dst[7] = f[5];  dst[8] = '\0';
}

/* ---------------------------------------------------------------------------
 * UTC date "ddmmyy" -> "dd/mm/yy"  (RMC order)
 * ------------------------------------------------------------------------- */
static void m_parse_date(const char *f, uint8 len, char *dst, uint8 dst_size)
{
    if ((dst_size < 9U) || (len < 6U)) { return; }

    dst[0] = f[0];  dst[1] = f[1];  dst[2] = '/';
    dst[3] = f[2];  dst[4] = f[3];  dst[5] = '/';
    dst[6] = f[4];  dst[7] = f[5];  dst[8] = '\0';
}

/* ---------------------------------------------------------------------------
 * NMEA checksum: XOR of all chars between '$' and '*' compared to "*.hh"
 * ------------------------------------------------------------------------- */
static bool m_nmea_checksum_ok(const char *line, bool *has_checksum)
{
    const uint8 n = (uint8)strlen(line);
    uint8 calc  = 0U;
    uint8 hi    = 0U;
    uint8 lo    = 0U;
    uint8 i;

    *has_checksum = false;
    for (i = 1U; i < n; i++)
    {
        if (ASCII_ASTERISK == (uint8)line[i])
        {
            if ((int)(n - i) < 3) { return false; }         /* Truncated "*.hh"   */
            if (!m_hex_to_uint8(line[i + 1U], &hi)) { return false; }
            if (!m_hex_to_uint8(line[i + 2U], &lo)) { return false; }
            *has_checksum = true;
            return (calc == (uint8)((hi << 4) | lo));
        }
        calc ^= (uint8)line[i];
    }

    return true;    /* No '*' terminator: no checksum to validate */
}

/* ---------------------------------------------------------------------------
 * Sentence-Type resolver (talker-agnostic: $GP / $GN / $GL / $BD / $GA ...)
 * ------------------------------------------------------------------------- */
static uint8 m_nmea_type(const char *line)
{
    const char *t = &line[3];    /* Skip "$XX" */

    if ((line[0] != ASCII_DOLLAR) || (strlen(line) < 7U)) { return NMEA_TYPE_UNKNOWN; }

    if (('G' == t[0]) && ('G' == t[1]) && ('A' == t[2])) { return NMEA_TYPE_GGA; }
    if (('R' == t[0]) && ('M' == t[1]) && ('C' == t[2])) { return NMEA_TYPE_RMC; }
    if (('G' == t[0]) && ('L' == t[1]) && ('L' == t[2])) { return NMEA_TYPE_GLL; }
    if (('V' == t[0]) && ('T' == t[1]) && ('G' == t[2])) { return NMEA_TYPE_VTG; }
    if (('G' == t[0]) && ('S' == t[1]) && ('V' == t[2])) { return NMEA_TYPE_GSV; }

    return NMEA_TYPE_UNKNOWN;
}

/* ---------------------------------------------------------------------------
 * GGA - essential fix data
 * $--GGA,hhmmss.ss,llll.llll,a,yyyyy.yyyy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
 * ------------------------------------------------------------------------- */
static void m_parse_gga(const char *line)
{
    sim7600_gps_data_t *d = &m_gps.data;
    const char *f1; uint8 n1 = 0U;   /* latitude      */
    const char *f2; uint8 n2 = 0U;   /* N/S           */
    const char *f3; uint8 n3 = 0U;   /* longitude     */
    const char *f4; uint8 n4 = 0U;   /* E/W           */
    const char *f5; uint8 n5 = 0U;   /* fix quality   */
    const char *f6; uint8 n6 = 0U;   /* sat used      */
    const char *f7; uint8 n7 = 0U;   /* hdop          */
    const char *f8; uint8 n8 = 0U;   /* altitude m    */
    const char *f0; uint8 n0 = 0U;   /* time          */
    int32 lat = 0;
    int32 lon = 0;

    f0 = m_nmea_field(line, 0U, &n0);
    f1 = m_nmea_field(line, 1U, &n1);
    f2 = m_nmea_field(line, 2U, &n2);
    f3 = m_nmea_field(line, 3U, &n3);
    f4 = m_nmea_field(line, 4U, &n4);
    f5 = m_nmea_field(line, 5U, &n5);
    f6 = m_nmea_field(line, 6U, &n6);
    f7 = m_nmea_field(line, 7U, &n7);
    f8 = m_nmea_field(line, 8U, &n8);

    if ((NULL == f0) || (NULL == f1) || (NULL == f2) || (NULL == f3) ||
        (NULL == f4) || (NULL == f5) || (NULL == f6))
    {
        m_gps.stats.nmea_parse_errors++;
        return;
    }

    if (n0 > 0U) { m_parse_time(f0, n0, d->utc_time, 12U); }

    /* Position block */
    if ((n1 > 0U) && (n2 > 0U) && (n3 > 0U) && (n4 > 0U))
    {
        if (m_parse_latlon(f1, n1, 2U, f2[0], &lat) &&
            m_parse_latlon(f3, n3, 3U, f4[0], &lon))
        {
            d->lat_e6 = lat;
            d->lon_e6 = lon;
            d->latlon_fresh = true;
            d->timestamp_ms = HAL_GetTick();
        }
        else
        {
            m_gps.stats.nmea_parse_errors++;
            return;
        }
    }

    /* Fix quality: parsed through a safe 16-bit temp */
    {
        uint16 q = 0U;
        if (m_dec_uint16(f5, n5, &q))
        {
            d->fix_quality = (uint8)q;
        }
    }

    if (n6 > 0U)
    {
        uint16 v = 0U;
        if (m_dec_uint16(f6, n6, &v)) { d->sats_used = v; }
    }

    if ((NULL != f7) && (n7 > 0U))
    {
        int32 v = 0;
        if (m_dec_scaled(f7, n7, 2U, &v)) { d->hdop_cd = (uint16)v; }
    }

    if ((NULL != f8) && (n8 > 0U))
    {
        int32 v = 0;
        if (m_dec_scaled(f8, n8, 2U, &v)) { d->alt_cm = v; }   /* meters*100 = cm */
    }
}

/* ---------------------------------------------------------------------------
 * RMC - recommended minimum (authoritative 'valid' flag)
 * $--RMC,hhmmss.ss,A,llll.llll,a,yyyyy.yyyy,a,x.x,x.x,ddmmyy,x.x,a,a*hh
 * ------------------------------------------------------------------------- */
static void m_parse_rmc(const char *line)
{
    sim7600_gps_data_t *d = &m_gps.data;
    const char *f0; uint8 n0 = 0U;   /* time        */
    const char *f1; uint8 n1 = 0U;   /* status      */
    const char *f2; uint8 n2 = 0U;   /* latitude    */
    const char *f3; uint8 n3 = 0U;   /* N/S         */
    const char *f4; uint8 n4 = 0U;   /* longitude   */
    const char *f5; uint8 n5 = 0U;   /* E/W         */
    const char *f6; uint8 n6 = 0U;   /* knots       */
    const char *f7; uint8 n7 = 0U;   /* course      */
    const char *f8; uint8 n8 = 0U;   /* date        */
    int32 lat = 0;
    int32 lon = 0;
    int32 spd = 0;
    int32 crs = 0;

    f0 = m_nmea_field(line, 0U, &n0);
    f1 = m_nmea_field(line, 1U, &n1);
    f2 = m_nmea_field(line, 2U, &n2);
    f3 = m_nmea_field(line, 3U, &n3);
    f4 = m_nmea_field(line, 4U, &n4);
    f5 = m_nmea_field(line, 5U, &n5);
    f6 = m_nmea_field(line, 6U, &n6);
    f7 = m_nmea_field(line, 7U, &n7);
    f8 = m_nmea_field(line, 8U, &n8);

    if ((NULL == f0) || (NULL == f1))
    {
        m_gps.stats.nmea_parse_errors++;
        return;
    }

    if (n0 > 0U) { m_parse_time(f0, n0, d->utc_time, 12U); }

    /* Status: 'A' = valid fix, 'V' = void */
    if (n1 > 0U)
    {
        d->valid = ('A' == f1[0]);
    }

    if ((n2 > 0U) && (n3 > 0U) && (n4 > 0U) && (n5 > 0U))
    {
        if (m_parse_latlon(f2, n2, 2U, f3[0], &lat) &&
            m_parse_latlon(f4, n4, 3U, f5[0], &lon))
        {
            d->lat_e6 = lat;
            d->lon_e6 = lon;
            d->latlon_fresh = true;
        }
        else
        {
            m_gps.stats.nmea_parse_errors++;
            return;
        }
    }

    if ((NULL != f6) && (n6 > 0U))
    {
        if (m_dec_scaled(f6, n6, 2U, &spd))
        {
            d->speed_cknots = spd;                      /* centi-knots        */
            d->speed_ckmh   = (int32)(((uint32)spd * 1852UL) / 1000UL);  /* *1.852 */
        }
    }

    if ((NULL != f7) && (n7 > 0U))
    {
        if (m_dec_scaled(f7, n7, 2U, &crs))
        {
            d->course_cdeg = crs;
        }
    }

    if ((NULL != f8) && (n8 > 0U))
    {
        m_parse_date(f8, n8, d->utc_date, 10U);
    }

    d->timestamp_ms = HAL_GetTick();
}

/* ---------------------------------------------------------------------------
 * GLL - latitude/longitude + status
 * $--GLL,llll.ll,a,yyyyy.yyyy,a,hhmmss.ss,A,a*hh
 * ------------------------------------------------------------------------- */
static void m_parse_gll(const char *line)
{
    sim7600_gps_data_t *d = &m_gps.data;
    const char *f0; uint8 n0 = 0U;   /* latitude   */
    const char *f1; uint8 n1 = 0U;   /* N/S        */
    const char *f2; uint8 n2 = 0U;   /* longitude  */
    const char *f3; uint8 n3 = 0U;   /* E/W        */
    const char *f4; uint8 n4 = 0U;   /* time       */
    const char *f5; uint8 n5 = 0U;   /* status     */
    int32 lat = 0;
    int32 lon = 0;

    f0 = m_nmea_field(line, 0U, &n0);
    f1 = m_nmea_field(line, 1U, &n1);
    f2 = m_nmea_field(line, 2U, &n2);
    f3 = m_nmea_field(line, 3U, &n3);
    f4 = m_nmea_field(line, 4U, &n4);
    f5 = m_nmea_field(line, 5U, &n5);

    if ((NULL == f0) || (NULL == f2))
    {
        m_gps.stats.nmea_parse_errors++;
        return;
    }

    if ((n0 > 0U) && (n1 > 0U) && (n2 > 0U) && (n3 > 0U))
    {
        if (m_parse_latlon(f0, n0, 2U, f1[0], &lat) &&
            m_parse_latlon(f2, n2, 3U, f3[0], &lon))
        {
            d->lat_e6 = lat;
            d->lon_e6 = lon;
            d->latlon_fresh = true;
        }
        else
        {
            m_gps.stats.nmea_parse_errors++;
            return;
        }
    }

    if ((NULL != f4) && (n4 > 0U)) { m_parse_time(f4, n4, d->utc_time, 12U); }

    if ((NULL != f5) && (n5 > 0U))
    {
        d->valid = ('A' == f5[0]);
    }

    d->timestamp_ms = HAL_GetTick();
}

/* ---------------------------------------------------------------------------
 * VTG - course & speed
 * $--VTG,x.x,T,,.,M,x.x,N,x.x,K,a*hh
 * ------------------------------------------------------------------------- */
static void m_parse_vtg(const char *line)
{
    sim7600_gps_data_t *d = &m_gps.data;
    const char *f0; uint8 n0 = 0U;   /* course true deg */
    const char *f4; uint8 n4 = 0U;   /* speed knots     */
    const char *f6; uint8 n6 = 0U;   /* speed km/h      */
    int32 v = 0;

    f0 = m_nmea_field(line, 0U, &n0);
    f4 = m_nmea_field(line, 4U, &n4);
    f6 = m_nmea_field(line, 6U, &n6);

    if (NULL == f0) { m_gps.stats.nmea_parse_errors++; return; }

    if ((n0 > 0U) && m_dec_scaled(f0, n0, 2U, &v))
    {
        d->course_cdeg = v;
    }

    if ((NULL != f4) && (n4 > 0U) && m_dec_scaled(f4, n4, 2U, &v))
    {
        d->speed_cknots = v;
        d->speed_ckmh   = (int32)(((uint32)v * 1852UL) / 1000UL);
    }
    else if ((NULL != f6) && (n6 > 0U) && m_dec_scaled(f6, n6, 2U, &v))
    {
        d->speed_ckmh  = v;
        d->speed_cknots = (int32)(((uint32)v * 1000UL) / 1852UL);   /* /1.852 */
    }

    d->timestamp_ms = HAL_GetTick();
}

/* ---------------------------------------------------------------------------
 * GSV - satellites in view (uses first message of each burst)
 * $--GSV,total,msg#,sats-in-view[,...*hh
 * ------------------------------------------------------------------------- */
static void m_parse_gsv(const char *line)
{
    const char *f0; uint8 n0 = 0U;   /* message count */
    const char *f1; uint8 n1 = 0U;   /* message #     */
    const char *f2; uint8 n2 = 0U;   /* in view       */
    uint16 in_view = 0U;
    uint16 msg_num = 0U;

    f0 = m_nmea_field(line, 0U, &n0);
    f1 = m_nmea_field(line, 1U, &n1);
    f2 = m_nmea_field(line, 2U, &n2);

    if ((NULL == f1) || (NULL == f2) || !m_dec_uint16(f1, n1, &msg_num))
    {
        m_gps.stats.nmea_parse_errors++;
        return;
    }

    /* Only the first GSV of a burst carries the authoritative in-view count */
    if (1U == msg_num && m_dec_uint16(f2, n2, &in_view))
    {
        m_gps.data.sats_visible = in_view;
    }

    (void)f0;
    (void)n0;
}

/* ---------------------------------------------------------------------------
 * NMEA dispatcher
 * ------------------------------------------------------------------------- */
static void m_parse_nmea(const char *line)
{
    bool has_checksum = false;
    uint8 type;

    if (m_gps.checksum_strict && !m_nmea_checksum_ok(line, &has_checksum))
    {
        m_gps.stats.nmea_bad_checksum++;
        return;    /* Strict mode rejects bad/missing checksum when expected */
    }

    type = m_nmea_type(line);

    switch (type)
    {
        case NMEA_TYPE_GGA: m_parse_gga(line); break;
        case NMEA_TYPE_RMC: m_parse_rmc(line); break;
        case NMEA_TYPE_GLL: m_parse_gll(line); break;
        case NMEA_TYPE_VTG: m_parse_vtg(line); break;
        case NMEA_TYPE_GSV: m_parse_gsv(line); break;
        default:
            m_gps.stats.nmea_unknown++;
            break;
    }

    if (NMEA_TYPE_UNKNOWN != type)
    {
        m_gps.stats.nmea_sentences++;
    }
}

/* ---------------------------------------------------------------------------
 * Completed-line dispatcher: NMEA vs AT-status vs informational
 * ------------------------------------------------------------------------- */
static void m_process_line(void)
{
    const char *l = &m_gps.line[0];

    m_gps.line[m_gps.line_len] = '\0';

    if (0U == m_gps.line_len) { return; }

    /* Raw capture: last completed line of any kind (Phase 3 debug dump) */
    (void)strncpy(m_gps.last_line, l, SIM7600_GPS_LINE_MAX_BYTES - 1U);
    m_gps.last_line[SIM7600_GPS_LINE_MAX_BYTES - 1U] = '\0';
    m_gps.last_line_fresh = true;

    if (ASCII_DOLLAR == (uint8)l[0])
    {
        m_parse_nmea(l);
        return;
    }

    /* AT-side lines - classified only during an active exchange */
    if (GPS_AT_WAIT == m_gps.at_state)
    {
        if (0 == strncmp(l, STATUS_TOKEN_OK, 2U))
        {
            m_gps.at_state = GPS_AT_DONE_OK;
        }
        else if ((0 == strncmp(l, STATUS_TOKEN_ERROR, 5U)) ||
                 (0 == strncmp(l, STATUS_TOKEN_CME_ERROR, 11U)) ||
                 (0 == strncmp(l, STATUS_TOKEN_CMS_ERROR, 11U)))
        {
            m_gps.at_state = GPS_AT_DONE_FAIL;
        }
        else
        {
            /* Informational line (echo / "+CGNSSPWR: ..." etc.):
             * appended to the caller's response buffer when present */
            if (NULL != m_gps.at_resp)
            {
                const uint16 room = (uint16)(m_gps.at_resp_size - m_gps.at_resp_len - 2U);
                if ((uint16)strlen(l) <= room)
                {
                    (void)strncpy(&m_gps.at_resp[m_gps.at_resp_len], l, (size_t)strlen(l));
                    m_gps.at_resp_len = (uint16)(m_gps.at_resp_len + strlen(l));
                    m_gps.at_resp[m_gps.at_resp_len] = '\n';
                    m_gps.at_resp_len++;
                    m_gps.at_resp[m_gps.at_resp_len] = '\0';
                }
            }
        }
    }
}

/* ---------------------------------------------------------------------------
 * Feed a single byte into the line assembler
 * ------------------------------------------------------------------------- */
static void m_feed_byte(uint8 b)
{
    if ((ASCII_CR == b) || (ASCII_LF == b))
    {
        if (0U != m_gps.line_len)
        {
            m_process_line();
            m_gps.line_len   = 0U;
            m_gps.discarding = false;
        }
        return;
    }

    if (m_gps.discarding) { return; }    /* Drain mode: skip until CR/LF */

    if (m_gps.line_len >= (SIM7600_GPS_LINE_MAX_BYTES - 1U))
    {
        m_gps.stats.line_overflows++;
        m_gps.discarding = true;
        return;
    }

    m_gps.line[m_gps.line_len++] = (char)b;
}

/* ---------------------------------------------------------------------------
 * Drain whatever the UART already holds (non-blocking)
 * ------------------------------------------------------------------------- */
static void m_pump(void)
{
    uint8  chunk[32];
    uint16 n = 0U;

    while (E_OK == sim7600_port_recv(m_gps.port_copy.uart_id, &chunk[0], 32U, &n, 0UL))
    {
        uint16 i;
        for (i = 0U; i < n; i++)
        {
            m_feed_byte(chunk[i]);
        }
        if (n < 32U) { break; }    /* Buffer underflow: nothing waiting */
    }
}

/******************* Section 7 : Public APIs Implementation *******************/

Std_ReturnType sim7600_gps_init(const sim7600_gps_cfg_t *cfg)
{
    if ((NULL == cfg) || (NULL == cfg->port_cfg))
    {
        return E_NOT_OK;
    }

    /* Reset the layer */
    memset(&m_gps, 0, sizeof(m_gps));
    m_gps.port_copy        = *(cfg->port_cfg);    /* Private lifetime-safe copy */
    m_gps.checksum_strict  = cfg->checksum_strict;
    m_gps.at_state         = GPS_AT_IDLE;

    /* Bring up the UART core */
    if (E_OK != sim7600_port_init(&m_gps.port_copy))
    {
        return E_NOT_OK;    /* Detailed reason lives in port's last_status */
    }

    m_gps.init = true;

    if (cfg->auto_power_on)
    {
        (void)sim7600_gps_gnss_on(SIM7600_GPS_GNSS_ON_TIMEOUT_MS);
    }

    return E_OK;
}

Std_ReturnType sim7600_gps_deinit(bool gnss_power_off)
{
    Std_ReturnType ret = E_OK;

    if (!m_gps.init)
    {
        return E_NOT_OK;
    }

    if (gnss_power_off)
    {
        (void)sim7600_gps_gnss_off(SIM7600_GPS_AT_TIMEOUT_MS);
    }

    if (E_OK != sim7600_port_deinit(m_gps.port_copy.uart_id))
    {
        ret = E_NOT_OK;
    }

    memset(&m_gps, 0, sizeof(m_gps));
    return ret;
}

bool sim7600_gps_is_init(void)
{
    return m_gps.init;
}

void sim7600_gps_task(void)
{
    if (!m_gps.init) { return; }
    m_pump();
}

Std_ReturnType sim7600_gps_gnss_on(uint32 per_cmd_timeout_ms)
{
    Std_ReturnType ret = E_OK;

    if (!m_gps.init)
    {
        return E_NOT_OK;
    }

#if SIM7600_GPS_USE_ECHO_OFF
    if (!m_gps.echo_off_done)
    {
        (void)sim7600_gps_send_at(SIM7600_GPS_CMD_ECHO_OFF, NULL, 0U, per_cmd_timeout_ms);
        m_gps.echo_off_done = true;
    }
#endif

    (void)sim7600_port_rx_flush(m_gps.port_copy.uart_id);

    if (E_OK != sim7600_gps_send_at(SIM7600_GPS_CMD_GNSS_POWER_ON, NULL, 0U, per_cmd_timeout_ms))
    {
        ret = E_NOT_OK;
    }

    if (E_OK != sim7600_gps_send_at(SIM7600_GPS_CMD_NMEA_OUT_ON, NULL, 0U, per_cmd_timeout_ms))
    {
        ret = E_NOT_OK;
    }

    return ret;
}

Std_ReturnType sim7600_gps_gnss_off(uint32 per_cmd_timeout_ms)
{
    Std_ReturnType ret = E_OK;

    if (!m_gps.init)
    {
        return E_NOT_OK;
    }

    if (E_OK != sim7600_gps_send_at(SIM7600_GPS_CMD_NMEA_OUT_OFF, NULL, 0U, per_cmd_timeout_ms))
    {
        ret = E_NOT_OK;
    }

    if (E_OK != sim7600_gps_send_at(SIM7600_GPS_CMD_GNSS_POWER_OFF, NULL, 0U, per_cmd_timeout_ms))
    {
        ret = E_NOT_OK;
    }

    return ret;
}

Std_ReturnType sim7600_gps_wait_for_fix(uint32 timeout_ms)
{
    uint32 start;

    if (!m_gps.init)
    {
        return E_NOT_OK;
    }

    /* timeout_ms == 0 -> use the default fix-wait ceiling from cfg */
    const uint32 wait_ms = (0UL == timeout_ms) ?
                            SIM7600_GPS_FIX_WAIT_TIMEOUT_MS : timeout_ms;

    start = HAL_GetTick();

    do
    {
        m_pump();

        if (m_gps.data.valid)
        {
            return E_OK;
        }

        /* Guard against the caller blocking forever with tick wrap-around */
    } while ((HAL_GetTick() - start) < wait_ms);

    return E_NOT_OK;
}

Std_ReturnType sim7600_gps_send_at(const char *cmd, char *resp, uint16 resp_size, uint32 timeout_ms)
{
    uint32 start;
    uint16 cmd_len = 0U;

    if (!m_gps.init || (NULL == cmd))
    {
        return E_NOT_OK;
    }

    /* Reset exchange context */
    m_gps.at_resp      = resp;
    m_gps.at_resp_size = resp_size;
    m_gps.at_resp_len  = 0U;
    if (NULL != resp) { resp[0] = '\0'; }

    m_gps.at_state = GPS_AT_WAIT;

    cmd_len = (uint16)strlen(cmd);

    /* Terminate AT commands with CR (LF optional per V.250; SIMCom accepts) */
    if (cmd_len > 0U)
    {
        (void)sim7600_port_send(m_gps.port_copy.uart_id, (const uint8 *)cmd, cmd_len, timeout_ms);
    }
    (void)sim7600_port_send(m_gps.port_copy.uart_id, (const uint8 *)"\r\n", 2U, timeout_ms);

    start = HAL_GetTick();

    while (GPS_AT_WAIT == m_gps.at_state)
    {
        m_pump();

        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            m_gps.stats.at_timeout_count++;
            m_gps.at_resp  = NULL;
            m_gps.at_resp_len  = 0U;
            m_gps.at_resp_size = 0U;
            m_gps.at_state = GPS_AT_IDLE;
            return E_NOT_OK;
        }
    }

    /* Capture the terminal state BEFORE resetting the exchange context */
    {
        const gps_at_state_t final_state = m_gps.at_state;

        if (GPS_AT_DONE_OK == final_state)
        {
            m_gps.stats.at_ok_count++;
        }
        else
        {
            m_gps.stats.at_error_count++;
        }

        m_gps.at_resp      = NULL;
        m_gps.at_resp_len  = 0U;
        m_gps.at_resp_size = 0U;
        m_gps.at_state     = GPS_AT_IDLE;

        return (GPS_AT_DONE_OK == final_state) ? E_OK : E_NOT_OK;
    }
}

Std_ReturnType sim7600_gps_get_data(sim7600_gps_data_t *out)
{
    if (!m_gps.init || (NULL == out))
    {
        return E_NOT_OK;
    }

    *out = m_gps.data;
    return E_OK;
}

bool sim7600_gps_fix_valid(void)
{
    return (m_gps.init && m_gps.data.valid);
}

Std_ReturnType sim7600_gps_get_raw_line(char *dst, uint16 dst_size)
{
    if (!m_gps.init || (NULL == dst) || (dst_size == 0U))
    {
        return E_NOT_OK;
    }

    if (!m_gps.last_line_fresh)
    {
        return E_NOT_OK;
    }

    (void)strncpy(dst, m_gps.last_line, (size_t)((dst_size < SIM7600_GPS_LINE_MAX_BYTES) ? dst_size : SIM7600_GPS_LINE_MAX_BYTES) - 1U);
    dst[((dst_size < SIM7600_GPS_LINE_MAX_BYTES) ? dst_size : SIM7600_GPS_LINE_MAX_BYTES) - 1U] = '\0';
    m_gps.last_line_fresh = false;
    return E_OK;
}

Std_ReturnType sim7600_gps_get_stats(sim7600_gps_stats_t *out)
{
    if ((NULL == out) || !m_gps.init)
    {
        return E_NOT_OK;
    }

    *out = m_gps.stats;
    return E_OK;
}

Std_ReturnType sim7600_gps_reset_stats(void)
{
    if (!m_gps.init)
    {
        return E_NOT_OK;
    }

    memset(&m_gps.stats, 0, sizeof(m_gps.stats));
    return E_OK;
}
