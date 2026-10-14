/******************* Section 0 : File Guard *******************/

#ifndef __ECUAL_HX711_H
#define __ECUAL_HX711_H

/******************* Section 1 : Includes *******************/

#include "../../MCAL/mcal_std_types.h"

/******************* Section 2 : Macros Declarations *******************/

/* --- 50Kg Load Cell Ratings & Thresholds --- */
#define HX711_RATED_CAPACITY_KG_DEFAULT     (50.0f)   /* 50Kg rated maximum capacity */
#define HX711_OVERLOAD_MARGIN_KG_DEFAULT    (2.0f)    /* 2Kg safety margin before hard overload */
#define HX711_UNDERLOAD_LIMIT_KG            (-0.5f)   /* Anomaly/negative weight detection threshold */

/************** Section 3 : Macros Functions Declarations **************/


/************** Section 4 : User Defined Data Types & Variables Declarations **************/

typedef enum {
    HX711_GAIN_128 = 1, /* Channel A, Gain 128 (25 pulses) */
    HX711_GAIN_32  = 2, /* Channel B, Gain 32  (26 pulses) */
    HX711_GAIN_64  = 3  /* Channel A, Gain 64  (27 pulses) */
} hx711_gain_t;

typedef struct {
    gpio_num_t dout_pin;
    gpio_num_t sck_pin;
    hx711_gain_t gain;
    int32 offset;              /* Tare zero-point baseline value */
    float scale;               /* Calibration factor: raw ADC counts per 1.0 kg */
    float max_capacity_kg;     /* Maximum rated load cell capacity (50.0 kg) */
    float overload_margin_kg;  /* Permissible margin above max capacity (e.g. 2.0 kg) */
} hx711_config_t;

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/* Core HX711 Driver APIs */
Std_ReturnType ecual_hx711_init(hx711_config_t *hx711);
Std_ReturnType ecual_hx711_is_ready(hx711_config_t *hx711, bool *is_ready);
Std_ReturnType ecual_hx711_read_raw(hx711_config_t *hx711, int32 *raw_value);
Std_ReturnType ecual_hx711_read_average(hx711_config_t *hx711, uint8 times, int32 *avg_value);
Std_ReturnType ecual_hx711_read_data(hx711_config_t *hx711, int32 *raw_value, float *weight, uint8 times);
Std_ReturnType ecual_hx711_tare(hx711_config_t *hx711, uint8 times);
Std_ReturnType ecual_hx711_get_units(hx711_config_t *hx711, uint8 times, float *weight);
Std_ReturnType ecual_hx711_power_down(hx711_config_t *hx711);
Std_ReturnType ecual_hx711_power_up(hx711_config_t *hx711);

/* 50Kg Load Cell Custom APIs */
Std_ReturnType ecual_hx711_get_weight_kg(hx711_config_t *hx711, uint8 times, float *weight_kg);
Std_ReturnType ecual_hx711_get_weight_g(hx711_config_t *hx711, uint8 times, float *weight_g);
Std_ReturnType ecual_hx711_is_overloaded(hx711_config_t *hx711, uint8 times, bool *is_overloaded);
Std_ReturnType ecual_hx711_calibrate_scale(hx711_config_t *hx711, float known_weight_kg, uint8 times, float *calculated_scale);
Std_ReturnType ecual_hx711_set_scale(hx711_config_t *hx711, float scale);
Std_ReturnType ecual_hx711_set_offset(hx711_config_t *hx711, int32 offset);

#endif /* __ECUAL_HX711_H */
