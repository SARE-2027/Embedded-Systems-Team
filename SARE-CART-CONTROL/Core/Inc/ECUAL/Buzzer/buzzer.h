/******************* Section 0 : File Guard *******************/

#ifndef __BUZZER_H
#define __BUZZER_H

/******************* Section 1 : Includes *******************/

#include "../../MCAL/mcal_std_types.h"

/******************* Section 2 : Macros Declarations *******************/

/* --- Driver Version --- */
#define BUZZER_VERSION_MAJOR          (1U)
#define BUZZER_VERSION_MINOR          (0U)
#define BUZZER_VERSION_PATCH          (0U)

/* --- Pin Encoding Macro: BUZZER_PIN(port, pin_number) ---
 * Example: BUZZER_PIN(A, 8) -> PA8, BUZZER_PIN(B, 4) -> PB4
 */
#define BUZZER_PORT_A                 (0U)
#define BUZZER_PORT_B                 (1U)
#define BUZZER_PORT_C                 (2U)
#define BUZZER_PORT_D                 (3U)
#define BUZZER_PORT_E                 (4U)
#define BUZZER_PORT_H                 (7U)
#define BUZZER_PIN(port, num)         ((uint8)(((uint8)BUZZER_PORT_##port << 4U) | ((uint8)(num) & 0x0FU)))

/* --- Musical Note Frequencies in Hertz (Hz) --- */
#define NOTE_REST                     (0U)
#define NOTE_C4                       (262U)
#define NOTE_CS4                      (277U)
#define NOTE_D4                       (294U)
#define NOTE_DS4                      (311U)
#define NOTE_E4                       (330U)
#define NOTE_F4                       (349U)
#define NOTE_FS4                      (370U)
#define NOTE_G4                       (392U)
#define NOTE_GS4                      (415U)
#define NOTE_A4                       (440U)
#define NOTE_AS4                      (466U)
#define NOTE_B4                       (494U)
#define NOTE_C5                       (523U)
#define NOTE_CS5                      (554U)
#define NOTE_D5                       (587U)
#define NOTE_DS5                      (622U)
#define NOTE_E5                       (659U)
#define NOTE_F5                       (698U)
#define NOTE_FS5                      (740U)
#define NOTE_G5                       (784U)
#define NOTE_GS5                      (831U)
#define NOTE_A5                       (880U)
#define NOTE_AS5                      (932U)
#define NOTE_B5                       (988U)
#define NOTE_C6                       (1047U)
#define NOTE_CS6                      (1109U)
#define NOTE_D6                       (1175U)
#define NOTE_DS6                      (1245U)
#define NOTE_E6                       (1319U)
#define NOTE_F6                       (1397U)
#define NOTE_FS6                      (1480U)
#define NOTE_G6                       (1568U)
#define NOTE_GS6                      (1661U)
#define NOTE_A6                       (1760U)
#define NOTE_B6                       (1976U)
#define NOTE_C7                       (2093U)

#define BUZZER_MAX_MELODY_NOTES       (16U)

/******************* Section 3 : Macros Functions Declarations **************/

/* (none) */

/************** Section 4 : User Defined Data Types & Variables Declarations **************/

/* Hardware Buzzer Type */
typedef enum {
    BUZZER_TYPE_ACTIVE = 0,       /* Active buzzer (generates fixed tone on DC High) */
    BUZZER_TYPE_PASSIVE           /* Passive buzzer (requires frequency square wave) */
} buzzer_type_t;

/* System Event Melody Presets */
typedef enum {
    BUZZER_MELODY_NONE = 0,
    BUZZER_MELODY_ITEM_ADDED,     /* Upbeat ascending chime (C6 -> E6): Item put into cart */
    BUZZER_MELODY_ITEM_REMOVED,   /* Descending chime (E6 -> C6): Item removed from cart */
    BUZZER_MELODY_CART_ERROR,     /* Dual low harsh buzz (A4 x 2): Weight mismatch / Error */
    BUZZER_MELODY_SUCCESS,        /* Happy 4-tone victory arpeggio: Checkout / Payment OK */
    BUZZER_MELODY_BOOT,           /* Warm power-on welcome chime (C5 -> G5 -> C6) */
    BUZZER_MELODY_CLICK           /* Ultra-short 25ms crisp keypress tick */
} buzzer_preset_t;

/* Single Note definition: Frequency in Hz and Duration in ms */
typedef struct {
    uint16 freq_hz;               /* Frequency in Hz (0 = NOTE_REST for pause) */
    uint16 duration_ms;           /* Duration in milliseconds */
} buzzer_note_t;

/* Configuration Struct */
typedef struct {
    uint8         pin;            /* BUZZER_PIN(port, pin) */
    buzzer_type_t type;           /* Active or Passive */
    bool          active_high;    /* true = 3.3V active (default), false = GND active */
    TIM_TypeDef  *tim;            /* Optional Hardware Timer for PWM (e.g. TIM2, TIM3, TIM4) or NULL for GPIO */
    uint8         tim_channel;    /* Timer channel: 1, 2, 3, 4 (used if tim != NULL) */
} buzzer_cfg_t;

/* Driver Instance Handle */
typedef struct {
    GPIO_TypeDef *port;
    uint16        pin;
    buzzer_type_t type;
    bool          active_high;
    TIM_TypeDef  *tim;            /* Hardware Timer pointer or NULL */
    uint8         tim_channel;    /* 1, 2, 3, 4 */

    /* Melody sequencer state (non-blocking) */
    buzzer_note_t melody_queue[BUZZER_MAX_MELODY_NOTES];
    uint8         melody_len;
    uint8         current_note_idx;
    uint32        note_start_tick;
    bool          is_playing;
    bool          is_beeping;
    uint32        beep_end_tick;
} buzzer_t;

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/**
 * @brief  Initialize the Buzzer driver instance with specified pin, type, and polarity.
 * @param  bz   Pointer to instance handle.
 * @param  cfg  Pointer to configuration structure.
 * @retval E_OK on success, E_NOT_OK on invalid parameters.
 */
Std_ReturnType buzzer_init(buzzer_t *bz, const buzzer_cfg_t *cfg);

/**
 * @brief  Simple non-blocking beep for active or passive buzzer.
 * @param  bz           Pointer to instance handle.
 * @param  duration_ms  Duration in milliseconds.
 * @retval E_OK on success, E_NOT_OK on failure.
 */
Std_ReturnType buzzer_beep(buzzer_t *bz, uint16 duration_ms);

/**
 * @brief  Play a single tone of specific frequency and duration (non-blocking).
 * @param  bz           Pointer to instance handle.
 * @param  freq_hz      Frequency in Hz (e.g. NOTE_A4, NOTE_C6).
 * @param  duration_ms  Duration in milliseconds.
 * @retval E_OK on success, E_NOT_OK on failure.
 */
Std_ReturnType buzzer_play_tone(buzzer_t *bz, uint16 freq_hz, uint16 duration_ms);

/**
 * @brief  Play a built-in system sound effect (Item Added, Removed, Error, Checkout, etc.).
 * @param  bz      Pointer to instance handle.
 * @param  preset  One of buzzer_preset_t (e.g. BUZZER_MELODY_ITEM_ADDED).
 * @retval E_OK on success, E_NOT_OK on failure.
 */
Std_ReturnType buzzer_play_preset(buzzer_t *bz, buzzer_preset_t preset);

/**
 * @brief  Play a custom melody composed of an array of notes and durations (non-blocking).
 * @param  bz          Pointer to instance handle.
 * @param  notes       Pointer to array of buzzer_note_t.
 * @param  note_count  Number of notes (up to BUZZER_MAX_MELODY_NOTES).
 * @retval E_OK on success, E_NOT_OK on failure.
 */
Std_ReturnType buzzer_play_melody(buzzer_t *bz, const buzzer_note_t *notes, uint8 note_count);

/**
 * @brief  Immediately silence and stop any currently playing tone or melody.
 * @param  bz  Pointer to instance handle.
 * @retval E_OK on success.
 */
Std_ReturnType buzzer_stop(buzzer_t *bz);

/**
 * @brief  Check if the buzzer is currently playing a melody or beeping.
 * @param  bz  Pointer to instance handle.
 * @retval true if active, false if idle/silent.
 */
bool           buzzer_is_busy(const buzzer_t *bz);

/**
 * @brief  Non-blocking service task to advance melody notes asynchronously.
 *         Must be called periodically from ecual_task() or main while(1).
 * @param  bz  Pointer to instance handle.
 */
void           buzzer_task(buzzer_t *bz);

#endif /* __BUZZER_H */
