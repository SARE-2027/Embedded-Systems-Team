/******************* Section 0 : File Guard *******************/

#ifndef __HX711_H
#define __HX711_H

/******************* Section 1 : Includes *******************/

#include "../../MCAL/mcal_std_types.h"

/******************* Section 2 : Macros Declarations *******************/

/* --- Driver Version --- */
#define HX711_VERSION_MAJOR (2U)
#define HX711_VERSION_MINOR (1U)
#define HX711_VERSION_PATCH (0U)

/* --- Timeouts --- */
#define HX711_READ_TIMEOUT_MS                                                  \
  (200UL) /* HX711 @10SPS = 100ms/sample max; 200ms safe margin */

/* --- Anti-Creep & Advanced Filtering Configuration --- */
#define HX711_MEDIAN_WINDOW_SIZE                                               \
  (7U) /* 7-sample window for Olympic Trimmed-Mean filter    */
#define HX711_ZERO_DEADBAND_MG                                                 \
  (1500) /* Clamp near-zero within ±1.5 g to strictly 0 mg     */
#define HX711_AUTO_ZERO_TRACK_MS                                               \
  (1000UL) /* Interval (ms) for slow zero-point baseline tracking */
#define HX711_STABLE_DELTA_MG                                                  \
  (250) /* Max change per sample considered stable (0.25 g)   */
#define HX711_STABLE_SAMPLES_REQ                                               \
  (4U) /* Consecutive stable samples before locking reading  */
#define HX711_CREEP_BAND_MG                                                    \
  (1500) /* Creep guard band: keep locked within ±1.5 g        */
#define HX711_FAST_TRACK_DELTA_MG                                              \
  (1500) /* Delta above which adaptive filter snaps fast (1.5g)*/
#define HX711_DISPLAY_RESOLUTION_MG                                            \
  (100) /* 0.1g (100mg) scale division rounding for stability */

/* --- Generic Pin Selection ---
 *   HX711_PIN(B, 0) -> PB0,  HX711_PIN(C, 13) -> PC13
 *   Pin id layout: bits[7:4] = port (A=0 .. E=4, H=7), bits[3:0] = pin number
 * (0..15).
 */
#define HX711_PORT_A (0U)
#define HX711_PORT_B (1U)
#define HX711_PORT_C (2U)
#define HX711_PORT_D (3U)
#define HX711_PORT_E (4U)
#define HX711_PORT_H (7U)
#define HX711_PIN(port_letter, num)                                            \
  ((uint8)((HX711_PORT_##port_letter << 4U) | ((uint8)(num) & 0x0FU)))

/******************* Section 3 : Macros Functions Declarations **************/

/* (none) */

/************** Section 4 : User Defined Data Types & Variables Declarations
 * **************/

/* --- Input Channel / Gain Selection (value = number of extra clock pulses) ---
 */
typedef enum {
  HX711_GAIN_128 = 1, /* Channel A, gain 128 (default) */
  HX711_GAIN_32 = 2,  /* Channel B, gain 32            */
  HX711_GAIN_64 = 3   /* Channel A, gain 64            */
} hx711_gain_t;

typedef struct {
  uint8 sck_pin;
  uint8 dout_pin;
  hx711_gain_t gain;
} hx711_cfg_t;

/*
 *   offset            : raw counts at zero load (set by hx711_tare)
 *   scale_cpk         : raw counts per kilogram (set by hx711_calibrate /
 * hx711_set_scale) latest_raw        : last raw 24-bit sample seen by
 * hx711_task (debug / calibration aid) latest_weight_mg  : filtered weight in
 * milligrams
 */
typedef struct {
  GPIO_TypeDef *sck_port;
  uint16 sck_pin;
  GPIO_TypeDef *dout_port;
  uint16 dout_pin;
  hx711_gain_t gain;
  int32 offset;
  int32 scale_cpk;
  int32 latest_raw;
  int32 latest_weight_mg;
  bool latest_weight_valid;

  /* Multi-stage Hybrid Filter & Anti-Creep Engine */
  int32 median_buf[HX711_MEDIAN_WINDOW_SIZE];
  uint8 median_idx;
  uint8 median_count;
  int32 filtered_weight_mg;
  int32 prev_filtered_weight_mg;
  int32 stable_weight_mg;
  uint8 stable_count;
  bool is_stable;
  uint32 last_azt_tick;
} hx711_t;

/*************** Section 5 : Software Interfaces Declarations (APIs)
 * ***************/

Std_ReturnType hx711_init(hx711_t *h, const hx711_cfg_t *cfg);

/* Cooperative, non-blocking service task: consumes a sample only when ready. */
void hx711_task(hx711_t *h);

/* Most recent filtered weight in milligrams. false until the first sample after
 * init/tare/calibrate. */
bool hx711_get_latest_weight_mg(const hx711_t *h, int32 *weight_mg);

/* Check if the current reading is stable (anti-creep active). */
bool hx711_is_stable(const hx711_t *h);

/* Most recent raw sample taken by hx711_task. false until the first sample. */
bool hx711_get_latest_raw(const hx711_t *h, int32 *raw);

/* Restart the EMA filter (next sample becomes the new filter state). */
void hx711_reset_filter(hx711_t *h);

bool hx711_is_ready(const hx711_t *h);

Std_ReturnType hx711_read_raw(hx711_t *h, int32 *value, uint32 timeout_ms);
Std_ReturnType hx711_read_average(hx711_t *h, uint8 samples, int32 *value);

/* Zero point = average of 'samples' readings (cell must be unloaded). */
Std_ReturnType hx711_tare(hx711_t *h, uint8 samples);

/* Manually set scale (counts per kg, != 0) / offset. */
Std_ReturnType hx711_set_scale(hx711_t *h, int32 counts_per_kg);
Std_ReturnType hx711_set_offset(hx711_t *h, int32 offset);

/* Calibrate scale with a known weight (grams, != 0) on the cell. Call
 * hx711_tare() first. */
Std_ReturnType hx711_calibrate(hx711_t *h, uint32 known_weight_g,
                               uint8 samples);

Std_ReturnType hx711_get_counts(hx711_t *h, uint8 samples, int32 *counts);
Std_ReturnType hx711_get_weight_mg(hx711_t *h, uint8 samples, int32 *weight_mg);

Std_ReturnType hx711_set_gain(hx711_t *h, hx711_gain_t gain);

void hx711_power_down(hx711_t *h);
Std_ReturnType hx711_power_up(hx711_t *h);

#endif /* __HX711_H */
