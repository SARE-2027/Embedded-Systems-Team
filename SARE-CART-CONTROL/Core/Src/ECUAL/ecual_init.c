/******************* Section 0 : Includes *******************/

#include "../../Inc/ECUAL/ecual_init.h"

/******************* Section 1 : Variables Definitions *******************/

/* GPS Channel Pin & Hardware Mapping */
#define ECUAL_GPS_UART_ID (SIM7600_UART_1)
#define ECUAL_GPS_TX_PIN (SIM7600_PIN_PA9)
#define ECUAL_GPS_RX_PIN (SIM7600_PIN_PA10)
#define ECUAL_GPS_BAUD                                                         \
  (115200UL) /* SIM7600E-H factory default baud: 115200                        \
              */
#define ECUAL_GPS_RX_BUF_BYTES (1024U)
#define ECUAL_GPS_NVIC_PRI (8U)

/* HX711 Load Cell Pin & Calibration Mapping */
#define ECUAL_HX711_SCK_PIN                                                    \
  (HX711_PIN(B, 0)) /* any pin: HX711_PIN(<port letter>, <number>) */
#define ECUAL_HX711_DOUT_PIN (HX711_PIN(B, 1))
#define ECUAL_HX711_GAIN (HX711_GAIN_128)
#define ECUAL_HX711_SCALE_CPK                                                  \
  (122500) /* Calibrated with post-retighten Net RAW (5880 counts @ 48g):      \
              122.5 counts/g */
#define ECUAL_HX711_AUTO_TARE (1U)
#define ECUAL_HX711_TARE_SAMPLES (10U)

/* NFC Module Pin & Hardware Mapping */
#define ECUAL_NFC_SPI_ID (NFC_SPI_1)
#define ECUAL_NFC_MOSI_PIN (NFC_PIN(A, 7)) /* PA7: MOSI */
#define ECUAL_NFC_MISO_PIN (NFC_PIN(A, 6)) /* PA6: MISO */
#define ECUAL_NFC_SCK_PIN (NFC_PIN(A, 5))  /* PA5: SCK  */
#define ECUAL_NFC_CS_PIN (NFC_PIN(A, 4))   /* PA4: CS   */
#define ECUAL_NFC_BAUD_PRESCALER (NFC_SPI_BAUD_DIV_16)

/* RGB LED Strip Pin & Type Mapping */
#define ECUAL_RGB_RED_PIN (RGB_LED_PIN(B, 7)) /* PB7: Red channel (TIM4_CH2)   \
                                               */
#define ECUAL_RGB_GREEN_PIN                                                    \
  (RGB_LED_PIN(B, 8)) /* PB8: Green channel (TIM4_CH3) */
#define ECUAL_RGB_BLUE_PIN                                                     \
  (RGB_LED_PIN(B, 9)) /* PB9: Blue channel (TIM4_CH4) */
#define ECUAL_RGB_TYPE (RGB_LED_COMMON_CATHODE) /* or RGB_LED_COMMON_ANODE */

/* Buzzer Pin & Type Mapping */
#define ECUAL_BUZZER_PIN                                                       \
  (BUZZER_PIN(B, 10)) /* PB10: Buzzer pin (TIM2_CH3 / AF1) */
#define ECUAL_BUZZER_TYPE (BUZZER_TYPE_ACTIVE) /* Active buzzer default */
#define ECUAL_BUZZER_ACTIVE_HIGH (true)

/* User Push Button Pin & Mode Mapping */
#define ECUAL_BUTTON_PIN (BUTTON_PIN(C, 13))    /* PC13: User Button */
#define ECUAL_BUTTON_ACTIVE (BUTTON_ACTIVE_LOW) /* Low when pressed (pull-up)  \
                                                 */
#define ECUAL_BUTTON_PULL (BUTTON_PULL_UP)
#define ECUAL_BUTTON_MODE                                                      \
  (BUTTON_MODE_EXTI) /* Hardware EXTI line 13 / EXTI15_10_IRQn */

/* GPS RX Ring Buffer (storage for port IRQ driver) */
static uint8 m_gps_rx_buf[ECUAL_GPS_RX_BUF_BYTES];

/* Port Configuration for SIM7600 GPS */
static sim7600_port_config_t m_gps_port_cfg = {
    .uart_id = ECUAL_GPS_UART_ID,
    .tx_pin = ECUAL_GPS_TX_PIN,
    .rx_pin = ECUAL_GPS_RX_PIN,
    .baud_rate = ECUAL_GPS_BAUD,
    .word_len = SIM7600_WORD_LEN_8,
    .parity = SIM7600_PARITY_NONE,
    .stop_bits = SIM7600_STOP_BITS_1,
    .oversampling = SIM7600_OVERSAMPLE_16,
    .flow_control = SIM7600_FLOW_NONE,
    .cts_pin = SIM7600_PIN_NONE,
    .rts_pin = SIM7600_PIN_NONE,
    .rx_buffer = &m_gps_rx_buf[0],
    .rx_buffer_size = (uint16)ECUAL_GPS_RX_BUF_BYTES,
    .nvic_priority = ECUAL_GPS_NVIC_PRI};

/* SIM7600 GPS Layer Configuration */
static sim7600_gps_cfg_t m_gps_cfg = {.port_cfg = &m_gps_port_cfg,
                                      .auto_power_on = true,
                                      .checksum_strict = true};

/* HX711 Port Configuration & Instance */
static const hx711_cfg_t m_hx711_cfg = {.sck_pin = ECUAL_HX711_SCK_PIN,
                                        .dout_pin = ECUAL_HX711_DOUT_PIN,
                                        .gain = ECUAL_HX711_GAIN};
static hx711_t m_hx711;

/* NFC Configuration & Instance */
static const nfc_cfg_t m_nfc_cfg = {.spi_id = ECUAL_NFC_SPI_ID,
                                    .mosi_pin = ECUAL_NFC_MOSI_PIN,
                                    .miso_pin = ECUAL_NFC_MISO_PIN,
                                    .sck_pin = ECUAL_NFC_SCK_PIN,
                                    .cs_pin = ECUAL_NFC_CS_PIN,
                                    .baud_prescaler = ECUAL_NFC_BAUD_PRESCALER};
static nfc_t m_nfc;

/* RGB LED Configuration & Instance */
static const rgb_led_cfg_t m_rgb_cfg = {.red_pin = ECUAL_RGB_RED_PIN,
                                        .green_pin = ECUAL_RGB_GREEN_PIN,
                                        .blue_pin = ECUAL_RGB_BLUE_PIN,
                                        .type = ECUAL_RGB_TYPE};
static rgb_led_t m_rgb_led;

/* Buzzer Configuration & Instance */
static const buzzer_cfg_t m_buzzer_cfg = {
    .pin = ECUAL_BUZZER_PIN,
    .type = ECUAL_BUZZER_TYPE,
    .active_high = ECUAL_BUZZER_ACTIVE_HIGH,
    .tim = TIM2, /* PB10 Hardware PWM: TIM2_CH3 (AF1) */
    .tim_channel = 3U};
static buzzer_t m_buzzer;

/* Button Configuration & Instance */
static const button_cfg_t m_button_cfg = {.pin = ECUAL_BUTTON_PIN,
                                          .active_level = ECUAL_BUTTON_ACTIVE,
                                          .pull = ECUAL_BUTTON_PULL,
                                          .mode = ECUAL_BUTTON_MODE,
                                          .exti_preempt_priority = 6U,
                                          .exti_sub_priority = 0U,
                                          .debounce_ms = 20U,
                                          .long_press_ms = 1000U,
                                          .double_click_ms = 250U,
                                          .callback = NULL,
                                          .user_data = NULL};
static button_t m_button;

/* Track successful initialization before dispatching component tasks/APIs. */
static bool m_gps_initialized = false;
static bool m_hx711_initialized = false;
static bool m_nfc_initialized = false;
static bool m_rgb_initialized = false;
static bool m_buzzer_initialized = false;
static bool m_button_initialized = false;

/************** Section 2 : Helper Functions Declarations **************/

/*************** Section 3 : Software Interfaces Definitions (APIs)
 * ****************/

Std_ReturnType ecual_init(void) {
  Std_ReturnType Retval = E_OK;

  m_gps_initialized = false;
  m_hx711_initialized = false;
  m_nfc_initialized = false;
  m_rgb_initialized = false;
  m_buzzer_initialized = false;
  m_button_initialized = false;

  /* Initialize SIM7600 GPS Module */
  if (E_OK == sim7600_gps_init(&m_gps_cfg)) {
    m_gps_initialized = true;
  } else {
    Retval = E_NOT_OK;
  }

  /* Initialize HX711 Load Cell ADC */
  if (E_OK == hx711_init(&m_hx711, &m_hx711_cfg)) {
    m_hx711_initialized = true;
    (void)hx711_set_scale(&m_hx711, ECUAL_HX711_SCALE_CPK);
  } else {
    Retval = E_NOT_OK;
  }

  /* Initialize PN532 NFC Module */
  if (E_OK == nfc_init(&m_nfc, &m_nfc_cfg)) {
    m_nfc_initialized = true;
  } else {
    Retval = E_NOT_OK;
  }

  /* Initialize RGB LED Strip */
  if (E_OK == rgb_led_init(&m_rgb_led, &m_rgb_cfg)) {
    m_rgb_initialized = true;
    (void)rgb_led_set_color(&m_rgb_led, RGB_COLOR_OFF);
  } else {
    Retval = E_NOT_OK;
  }

  /* Initialize Buzzer */
  if (E_OK == buzzer_init(&m_buzzer, &m_buzzer_cfg)) {
    m_buzzer_initialized = true;
    (void)buzzer_stop(&m_buzzer);
  } else {
    Retval = E_NOT_OK;
  }

  /* Initialize User Push Button */
  if (E_OK == button_init(&m_button, &m_button_cfg)) {
    m_button_initialized = true;
  } else {
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
  if (m_nfc_initialized) {
    nfc_task(&m_nfc);
  }
  if (m_rgb_initialized) {
    rgb_led_task(&m_rgb_led);
  }
  if (m_buzzer_initialized) {
    buzzer_task(&m_buzzer);
  }
  if (m_button_initialized) {
    button_task_all();
  }
}

bool ecual_gps_is_ready(void) { return m_gps_initialized; }
bool ecual_hx711_is_ready(void) { return m_hx711_initialized; }
bool ecual_nfc_is_ready(void) { return m_nfc_initialized; }
bool ecual_rgb_led_is_ready(void) { return m_rgb_initialized; }
bool ecual_buzzer_is_ready(void) { return m_buzzer_initialized; }
bool ecual_button_is_ready(void) { return m_button_initialized; }

/* GPS APIs */
Std_ReturnType ecual_gps_get_data(sim7600_gps_data_t *out) {
  if (!m_gps_initialized || (NULL == out)) {
    return E_NOT_OK;
  }
  return sim7600_gps_get_data(out);
}

Std_ReturnType ecual_gps_get_diag(sim7600_gps_diag_t *out) {
  if (!m_gps_initialized || (NULL == out)) {
    return E_NOT_OK;
  }
  return sim7600_gps_get_diag(out);
}

/* HX711 APIs */
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

bool ecual_hx711_is_stable(void) {
  if (!m_hx711_initialized) {
    return false;
  }
  return hx711_is_stable(&m_hx711);
}

Std_ReturnType ecual_hx711_calibrate(uint32 known_weight_g, uint8 samples) {
  if (!m_hx711_initialized) {
    return E_NOT_OK;
  }
  return hx711_calibrate(&m_hx711, known_weight_g, samples);
}

Std_ReturnType ecual_hx711_set_scale(int32 counts_per_kg) {
  if (!m_hx711_initialized) {
    return E_NOT_OK;
  }
  return hx711_set_scale(&m_hx711, counts_per_kg);
}

Std_ReturnType ecual_hx711_set_offset(int32 offset) {
  if (!m_hx711_initialized) {
    return E_NOT_OK;
  }
  return hx711_set_offset(&m_hx711, offset);
}

Std_ReturnType ecual_hx711_get_scale(int32 *counts_per_kg) {
  if (!m_hx711_initialized || (NULL == counts_per_kg)) {
    return E_NOT_OK;
  }
  *counts_per_kg = m_hx711.scale_cpk;
  return E_OK;
}

Std_ReturnType ecual_hx711_get_offset(int32 *offset) {
  if (!m_hx711_initialized || (NULL == offset)) {
    return E_NOT_OK;
  }
  *offset = m_hx711.offset;
  return E_OK;
}

bool ecual_hx711_get_raw(int32 *raw) {
  if (!m_hx711_initialized || (NULL == raw)) {
    return false;
  }
  return hx711_get_latest_raw(&m_hx711, raw);
}

/* NFC APIs */
bool ecual_nfc_get_card(nfc_card_t *out_card) {
  if (!m_nfc_initialized || (NULL == out_card)) {
    return false;
  }
  return nfc_get_card(&m_nfc, out_card);
}

Std_ReturnType ecual_nfc_get_error(nfc_error_t *err) {
  if (NULL == err) {
    return E_NOT_OK;
  }
  *err = m_nfc.last_error;
  return E_OK;
}

/* RGB LED APIs */
Std_ReturnType ecual_rgb_set_color(rgb_color_t color) {
  if (!m_rgb_initialized) {
    return E_NOT_OK;
  }
  return rgb_led_set_color(&m_rgb_led, color);
}

Std_ReturnType ecual_rgb_set_rgb(uint8 r, uint8 g, uint8 b) {
  if (!m_rgb_initialized) {
    return E_NOT_OK;
  }
  return rgb_led_set_rgb(&m_rgb_led, r, g, b);
}

Std_ReturnType ecual_rgb_set_brightness(uint8 brightness_pct) {
  if (!m_rgb_initialized) {
    return E_NOT_OK;
  }
  return rgb_led_set_brightness(&m_rgb_led, brightness_pct);
}

Std_ReturnType ecual_rgb_turn_off(void) {
  if (!m_rgb_initialized) {
    return E_NOT_OK;
  }
  return rgb_led_off(&m_rgb_led);
}

rgb_color_t ecual_rgb_get_color(void) {
  if (!m_rgb_initialized) {
    return RGB_COLOR_OFF;
  }
  return rgb_led_get_color(&m_rgb_led);
}

Std_ReturnType ecual_rgb_get_rgb(rgb_color_val_t *out) {
  if (!m_rgb_initialized || (NULL == out)) {
    return E_NOT_OK;
  }
  return rgb_led_get_rgb(&m_rgb_led, out);
}

/* Buzzer APIs */
Std_ReturnType ecual_buzzer_beep(uint16 duration_ms) {
  if (!m_buzzer_initialized) {
    return E_NOT_OK;
  }
  return buzzer_beep(&m_buzzer, duration_ms);
}

Std_ReturnType ecual_buzzer_play_preset(buzzer_preset_t preset) {
  if (!m_buzzer_initialized) {
    return E_NOT_OK;
  }
  return buzzer_play_preset(&m_buzzer, preset);
}

Std_ReturnType ecual_buzzer_play_tone(uint16 freq_hz, uint16 duration_ms) {
  if (!m_buzzer_initialized) {
    return E_NOT_OK;
  }
  return buzzer_play_tone(&m_buzzer, freq_hz, duration_ms);
}

Std_ReturnType ecual_buzzer_play_melody(const buzzer_note_t *notes,
                                        uint8 note_count) {
  if (!m_buzzer_initialized) {
    return E_NOT_OK;
  }
  return buzzer_play_melody(&m_buzzer, notes, note_count);
}

Std_ReturnType ecual_buzzer_stop(void) {
  if (!m_buzzer_initialized) {
    return E_NOT_OK;
  }
  return buzzer_stop(&m_buzzer);
}

bool ecual_buzzer_is_busy(void) {
  if (!m_buzzer_initialized) {
    return false;
  }
  return buzzer_is_busy(&m_buzzer);
}

/* Push Button APIs */
button_state_t ecual_button_get_state(void) {
  if (!m_button_initialized) {
    return BUTTON_STATE_RELEASED;
  }
  return button_get_state(&m_button);
}

bool ecual_button_is_pressed(void) {
  if (!m_button_initialized) {
    return false;
  }
  return button_is_pressed(&m_button);
}

void ecual_button_set_callback(button_callback_t cb, void *user_data) {
  if (!m_button_initialized) {
    return;
  }
  button_set_callback(&m_button, cb, user_data);
}

/************** Section 4 : Helper Functions Definitions **************/
