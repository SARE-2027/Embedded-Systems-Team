/******************* Section 0 : File Guard *******************/

#ifndef __SIM7600_GPS_H
#define __SIM7600_GPS_H

/******************* Section 1 : Includes *******************/

#include "../../MCAL/mcal_std_types.h"

#include "sim7600_port.h"

/******************* Section 2 : Macros Declarations *******************/

/* --- Driver Version --- */
#define SIM7600_GPS_VERSION_MAJOR        (2U)
#define SIM7600_GPS_VERSION_MINOR        (0U)
#define SIM7600_GPS_VERSION_PATCH        (0U)

/* --- Line Assembly --- */
#define SIM7600_GPS_LINE_MAX_BYTES       (160U)   /* NMEA limit is 82 chars; AT responses may be longer */

/* --- AT Layer Timeouts --- */
#define SIM7600_GPS_AT_TIMEOUT_MS        (2000UL)   /* Generic AT command exchange          */
#define SIM7600_GPS_GNSS_ON_TIMEOUT_MS   (8000UL)   /* GNSS power-up sequence (module slow) */
#define SIM7600_GPS_FIX_WAIT_TIMEOUT_MS  (60000UL)  /* Default wait_for_fix ceiling (TTFF)  */

/* --- Boot Sequence (config-driven: swap command names here if a firmware
 *     variant of the SIM7600E-H uses different spellings; e.g. some units
 *     answer to AT+CGNSTST instead of AT+CGNSSTST) --- */
#define SIM7600_GPS_CMD_ECHO_OFF         "ATE0"
#define SIM7600_GPS_CMD_GNSS_POWER_ON    "AT+CGNSSPWR=1"
#define SIM7600_GPS_CMD_GNSS_POWER_OFF   "AT+CGNSSPWR=0"
#define SIM7600_GPS_CMD_NMEA_OUT_ON      "AT+CGNSSTST=1"   /* Route NMEA to UART port */
#define SIM7600_GPS_CMD_NMEA_OUT_OFF     "AT+CGNSSTST=0"

#define SIM7600_GPS_USE_ECHO_OFF         (1U)   /* ATE0 at first AT exchange   */

/* --- NMEA Sentence Types Parsed --- */
/*   GGA : fix quality, lat/lon, sats used, HDOP, altitude        */
/*   RMC : RCMS: time/date, status A/V, lat/lon, speed, course   */
/*   GLL : position + status                                     */
/*   VTG : course + speed (knots & km/h)                         */
/*   GSV : satellites in view                                    */

/******************* Section 3 : Macros Functions Declarations **************/

/* (none) */

/************** Section 4 : User Defined Data Types & Variables Declarations
 * **************/

/*
 * GNSS Layer Configuration.
 *
 *   port_cfg  : pointer to the sim7600_port_config_t that selects the USART
 *               channel / pins / baud. The GPS layer takes a private COPY of
 *               it, so a stack-local struct is safe to pass.
 *   auto_power_on : when true, sim7600_gps_init() runs the AT boot sequence
 *               (echo off + GNSS power on + NMEA output routed to the UART).
 *   checksum_strict : when true, NMEA sentences whose XOR checksum fails are
 *               dropped (recommended: true for fix data, they self-reprint).
 */
typedef struct {
    sim7600_port_config_t *port_cfg;
    bool                  auto_power_on;
    bool                  checksum_strict;
} sim7600_gps_cfg_t;

/*
 * Latest GNSS fix snapshot - fixed-point to stay lean on F401:
 *   lat_e6 / lon_e6  : micro-degrees (deg * 1e6). +N/-S, +E/-W.
 *                      e.g. 29915123 = 29.915123 deg N. Resolution ~0.1 m.
 *   alt_cm           : altitude in centimeters.
 *   speed_cknots     : ground speed * 100 (centi-knots), from RMC/VTG.
 *   speed_ckmh       : ground speed * 100 (centi-km/h)   = knots * 1.852.
 *   course_cdeg      : true course * 100.
 *   hdop_cd          : HDOP * 100 (e.g. 90 = 0.9).
 *   utc_*            : parse strings ("hh:mm:ss", "dd/mm/yy") ready to print.
 *   valid            : RMC status == 'A' (recommended fix-OK flag).
 *   fix_quality      : GGA field (0 none, 1 GPS, 2 DGPS, 4 RTK, 6 est.)
 *   timestamp_ms     : HAL_GetTick() of the last update of each field group.
 */
typedef struct {
    int32   lat_e6;
    int32   lon_e6;
    int32   alt_cm;
    int32   speed_cknots;
    int32   speed_ckmh;
    int32   course_cdeg;
    uint16  sats_used;            /* GGA: satellites in use   */
    uint16  sats_visible;         /* GSV: satellites in view  */
    uint16  hdop_cd;
    uint8   fix_quality;
    bool    valid;
    bool    latlon_fresh;         /* false after a position-less sentence set */
    char    utc_time[12];         /* "hh:mm:ss"   */
    char    utc_date[10];         /* "dd/mm/yy"   */
    uint32  timestamp_ms;         /* last data update tick */
} sim7600_gps_data_t;

/* --- GPS Layer Runtime Counters --- */
typedef struct {
    uint32 nmea_sentences;      /* Complete NMEA accepted (checksum ok)     */
    uint32 nmea_bad_checksum;   /* Rejected by checksum (when strict)       */
    uint32 nmea_parse_errors;   /* Known type but malformed fields          */
    uint32 nmea_unknown;        /* Type not in GGA/RMC/GLL/VTG/GSV set      */
    uint32 line_overflows;      /* Dropped bytes: line longer than buffer   */
    uint32 at_ok_count;
    uint32 at_error_count;
    uint32 at_timeout_count;
} sim7600_gps_stats_t;

/*
 * Default ready-to-use configuration:
 * binds the port layer default config (USART2 / PA2 / PA3 / 115200 per
 * sim7600_port_cfg.h), boots GNSS automatically, aims for strict checksums.
 */
extern const sim7600_gps_cfg_t sim7600_gps_default_cfg;

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/*
 * NOTE - Talker IDs are flexible: $GP.., $GN.., $GL.., $BD.., $GA.. are all
 * accepted, the layer keys on the sentence type (GGA/RMC/...) only.
 *
 * Concurrency model: ALL APIs may only be called from one execution context
 * (single main-loop thread). The UART RX itself is IRQ-driven at the port
 * layer, which is safe; parsing and AT flows run here, in the caller engine.
 */

/* --- Lifecycle --- */
Std_ReturnType sim7600_gps_init(const sim7600_gps_cfg_t *cfg);
Std_ReturnType sim7600_gps_deinit(bool gnss_power_off);
bool           sim7600_gps_is_init(void);

/* --- Periodic Service (call from main loop - never blocks) --- */
void           sim7600_gps_task(void);

/* --- GNSS Control --- */
Std_ReturnType sim7600_gps_gnss_on(uint32 per_cmd_timeout_ms);
Std_ReturnType sim7600_gps_gnss_off(uint32 per_cmd_timeout_ms);
Std_ReturnType sim7600_gps_wait_for_fix(uint32 timeout_ms);

/* --- AT Passthrough (for module bring-up / diagnostics) --- */
Std_ReturnType sim7600_gps_send_at(const char *cmd, char *resp, uint16 resp_size, uint32 timeout_ms);

/* --- Data Access --- */
Std_ReturnType sim7600_gps_get_data(sim7600_gps_data_t *out);
bool           sim7600_gps_fix_valid(void);

/* --- Raw Line Capture (last complete line, type-agnostic: Phase 3 feed) --- */
Std_ReturnType sim7600_gps_get_raw_line(char *dst, uint16 dst_size);

/* --- Diagnostics --- */
Std_ReturnType sim7600_gps_get_stats(sim7600_gps_stats_t *out);
Std_ReturnType sim7600_gps_reset_stats(void);

#endif /* __SIM7600_GPS_H */
