/******************* Section 0 : File Guard *******************/

#ifndef __HX711_H
#define __HX711_H

/******************* Section 1 : Includes *******************/

#include "../../MCAL/mcal_std_types.h"

/******************* Section 2 : Macros Declarations *******************/

/* --- Driver Version --- */
#define HX711_VERSION_MAJOR         (2U)
#define HX711_VERSION_MINOR         (0U)
#define HX711_VERSION_PATCH         (0U)

/* --- Timeouts --- */
#define HX711_READ_TIMEOUT_MS       (1000UL)   /* Data-ready wait: ~100 ms/sample @10 SPS, ~400 ms after power-up */

/* --- Generic Pin Selection ---
 *   Any GPIO can be used. Pick it at runtime with HX711_PIN(<port letter>, <pin number>):
 *       HX711_PIN(B, 0)  -> PB0        HX711_PIN(C, 13) -> PC13
 *   Pin id layout: bits[7:4] = port (A=0 .. E=4, H=7), bits[3:0] = pin number (0..15).
 */
#define HX711_PORT_A                (0U)
#define HX711_PORT_B                (1U)
#define HX711_PORT_C                (2U)
#define HX711_PORT_D                (3U)
#define HX711_PORT_E                (4U)
#define HX711_PORT_H                (7U)
#define HX711_PIN(port_letter, num) ((uint8)((HX711_PORT_##port_letter << 4U) | ((uint8)(num) & 0x0FU)))

/* --- Wiring Notes ---
 *   HX711 PD_SCK -> any GPIO (push-pull output, configured by hx711_init)
 *   HX711 DOUT   -> any GPIO (input + pull-up,  configured by hx711_init)
 *   HX711 VCC    -> 3.3 V (if 5 V, DOUT must be 5 V tolerant or level shifted)
 *
 *   Bit-banged: no timer or peripheral needed. Microsecond delays use the
 *   Cortex-M4 DWT cycle counter. Interrupts are masked during the 24+N clock
 *   pulses so PD_SCK is never held high long enough (> 60 us) to power the
 *   chip down.
 */

/******************* Section 3 : Macros Functions Declarations **************/

/* (none) */

/************** Section 4 : User Defined Data Types & Variables Declarations
 * **************/

/* --- Input Channel / Gain Selection (value = number of extra clock pulses) --- */
typedef enum {
    HX711_GAIN_128 = 1,      /* Channel A, gain 128 (default) */
    HX711_GAIN_32  = 2,      /* Channel B, gain 32            */
    HX711_GAIN_64  = 3       /* Channel A, gain 64            */
} hx711_gain_t;

/*
 * HX711 Configuration (hardware mapping chosen by the ECUAL gateway).
 *
 *   sck_pin   : PD_SCK line pin id, built with HX711_PIN(x, n)
 *   dout_pin  : DOUT line pin id,   built with HX711_PIN(x, n)
 *   gain      : channel / gain used from the first conversion
 */
typedef struct {
    uint8         sck_pin;
    uint8         dout_pin;
    hx711_gain_t  gain;
} hx711_cfg_t;

/*
 * HX711 Instance (one per load-cell amplifier). Fixed-point only:
 *
 *   offset       : raw counts at zero load (set by hx711_tare)
 *   scale_cpk    : raw counts per kilogram (set by hx711_calibrate / hx711_set_scale)
 *   weight       : reported in milligrams (int32 -> +/- 2147 kg range)
 */
typedef struct {
    GPIO_TypeDef *sck_port;
    uint16        sck_pin;
    GPIO_TypeDef *dout_port;
    uint16        dout_pin;
    hx711_gain_t  gain;         /* Gain used for the NEXT conversion */
    int32         offset;
    int32         scale_cpk;
    int32         latest_weight_mg;
    bool          latest_weight_valid;
} hx711_t;

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/* Initialise GPIOs + DWT counter and power the chip up. Returns E_NOT_OK on bad args. */
Std_ReturnType hx711_init(hx711_t *h, const hx711_cfg_t *cfg);

/* Cooperative, non-blocking service task: consumes a sample only when ready. */
void           hx711_task(hx711_t *h);

/* Return the most recent valid weight sample in milligrams. */
bool           hx711_get_latest_weight_mg(const hx711_t *h, int32 *weight_mg);

/* true if a conversion is ready (DOUT low) */
bool           hx711_is_ready(const hx711_t *h);

/* One raw signed 24-bit sample (sign-extended to int32). E_NOT_OK on timeout / bad args. */
Std_ReturnType hx711_read_raw(hx711_t *h, int32 *value, uint32 timeout_ms);

/* Average of 'samples' raw readings. */
Std_ReturnType hx711_read_average(hx711_t *h, uint8 samples, int32 *value);

/* Zero point = average of 'samples' readings (cell must be unloaded). */
Std_ReturnType hx711_tare(hx711_t *h, uint8 samples);

/* Manually set scale (raw counts per kg, must be != 0) / offset. */
Std_ReturnType hx711_set_scale(hx711_t *h, int32 counts_per_kg);
Std_ReturnType hx711_set_offset(hx711_t *h, int32 offset);

/* Calibrate scale with a known weight (grams, != 0) on the cell. Call hx711_tare() first. */
Std_ReturnType hx711_calibrate(hx711_t *h, uint32 known_weight_g, uint8 samples);

/* Raw average minus offset. */
Std_ReturnType hx711_get_counts(hx711_t *h, uint8 samples, int32 *counts);

/* Weight in milligrams: (raw - offset) * 1e6 / scale_cpk. */
Std_ReturnType hx711_get_weight_mg(hx711_t *h, uint8 samples, int32 *weight_mg);

/* Change gain/channel (takes effect on the sample AFTER the dummy read done here). */
Std_ReturnType hx711_set_gain(hx711_t *h, hx711_gain_t gain);

/* Power-down (PD_SCK high > 60 us) / wake-up (first valid sample after ~400 ms). */
void           hx711_power_down(hx711_t *h);
Std_ReturnType hx711_power_up(hx711_t *h);

#endif /* __HX711_H */
