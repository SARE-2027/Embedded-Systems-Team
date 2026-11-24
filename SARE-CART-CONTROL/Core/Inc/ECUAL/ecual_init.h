/******************* Section 0 : File Guard *******************/

#ifndef __ECUAL_INIT_H
#define __ECUAL_INIT_H

/******************* Section 1 : Includes *******************/

#include "../MCAL/mcal_std_types.h"
#include "Hx711/hx711.h"
#include "NFC/nfc.h"
#include "SIM7600EH/sim7600_gps.h"
#include "SIM7600EH/sim7600_port.h"
#include "RGB_LED/rgb_led.h"
#include "Buzzer/buzzer.h"
#include "Button/button.h"

/******************* Section 2 : Macros Declarations *******************/

/************** Section 3 : Macros Functions Declarations **************/

/************** Section 4 : User Defined Data Types & Variables Declarations **************/

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

Std_ReturnType ecual_init(void);

/* Cooperative service for all initialized ECUAL components. Call frequently in main loop. */
void ecual_task(void);

/* Application-facing ready checks */
bool ecual_gps_is_ready(void);
bool ecual_hx711_is_ready(void);
bool ecual_nfc_is_ready(void);
bool ecual_rgb_led_is_ready(void);
bool ecual_buzzer_is_ready(void);
bool ecual_button_is_ready(void);

/* GPS APIs */
Std_ReturnType ecual_gps_get_data(sim7600_gps_data_t *out);
Std_ReturnType ecual_gps_get_diag(sim7600_gps_diag_t *out);

/* HX711 Load Cell APIs */
Std_ReturnType ecual_hx711_tare(uint8 samples);
bool ecual_hx711_get_weight_mg(int32 *weight_mg);
bool ecual_hx711_is_stable(void);
Std_ReturnType ecual_hx711_calibrate(uint32 known_weight_g, uint8 samples);
Std_ReturnType ecual_hx711_set_scale(int32 counts_per_kg);
Std_ReturnType ecual_hx711_set_offset(int32 offset);
Std_ReturnType ecual_hx711_get_scale(int32 *counts_per_kg);
Std_ReturnType ecual_hx711_get_offset(int32 *offset);
bool ecual_hx711_get_raw(int32 *raw);

/* NFC APIs */
bool ecual_nfc_get_card(nfc_card_t *out_card);
Std_ReturnType ecual_nfc_get_error(nfc_error_t *err);

/* RGB LED APIs */
Std_ReturnType ecual_rgb_set_color(rgb_color_t color);
Std_ReturnType ecual_rgb_set_rgb(uint8 r, uint8 g, uint8 b);
Std_ReturnType ecual_rgb_set_brightness(uint8 brightness_pct);
Std_ReturnType ecual_rgb_turn_off(void);
rgb_color_t    ecual_rgb_get_color(void);
Std_ReturnType ecual_rgb_get_rgb(rgb_color_val_t *out);

/* Buzzer APIs */
Std_ReturnType ecual_buzzer_beep(uint16 duration_ms);
Std_ReturnType ecual_buzzer_play_preset(buzzer_preset_t preset);
Std_ReturnType ecual_buzzer_play_tone(uint16 freq_hz, uint16 duration_ms);
Std_ReturnType ecual_buzzer_play_melody(const buzzer_note_t *notes, uint8 note_count);
Std_ReturnType ecual_buzzer_stop(void);
bool           ecual_buzzer_is_busy(void);

/* Push Button APIs */
button_state_t ecual_button_get_state(void);
bool           ecual_button_is_pressed(void);
void           ecual_button_set_callback(button_callback_t cb, void *user_data);

#endif /* __ECUAL_INIT_H */
