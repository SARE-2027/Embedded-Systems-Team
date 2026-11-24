/******************* Section 0 : File Guard *******************/

#ifndef __RGB_LED_H
#define __RGB_LED_H

/******************* Section 1 : Includes *******************/

#include "../../MCAL/mcal_std_types.h"

/******************* Section 2 : Macros Declarations *******************/

/* --- Driver Version --- */
#define RGB_LED_VERSION_MAJOR         (1U)
#define RGB_LED_VERSION_MINOR         (0U)
#define RGB_LED_VERSION_PATCH         (0U)

/* --- Pin Encoding Macro: RGB_LED_PIN(port, pin_number) ---
 * Example: RGB_LED_PIN(A, 0) -> PA0, RGB_LED_PIN(B, 5) -> PB5
 */
#define RGB_LED_PORT_A                (0U)
#define RGB_LED_PORT_B                (1U)
#define RGB_LED_PORT_C                (2U)
#define RGB_LED_PORT_D                (3U)
#define RGB_LED_PORT_E                (4U)
#define RGB_LED_PORT_H                (7U)
#define RGB_LED_PIN(port, num)        ((uint8)(((uint8)RGB_LED_PORT_##port << 4U) | ((uint8)(num) & 0x0FU)))

/* --- Software PWM Resolution --- */
#define RGB_LED_PWM_STEPS             (32U)    /* 32-level software PWM for smooth color blending */

/******************* Section 3 : Macros Functions Declarations **************/

/* (none) */

/************** Section 4 : User Defined Data Types & Variables Declarations **************/

/* LED Hardware Connection Type */
typedef enum {
    RGB_LED_COMMON_CATHODE = 0,   /* Common GND: High (3.3V) = ON, Low (0V) = OFF */
    RGB_LED_COMMON_ANODE   = 1    /* Common VCC: Low (0V) = ON, High (3.3V) = OFF */
} rgb_led_type_t;

/* Extensive Palette of Preset Colors */
typedef enum {
    RGB_COLOR_OFF = 0,            /* Turn off completely */
    RGB_COLOR_RED,                /* Pure Red     (255,   0,   0) */
    RGB_COLOR_GREEN,              /* Pure Green   (  0, 255,   0) */
    RGB_COLOR_BLUE,               /* Pure Blue    (  0,   0, 255) */
    RGB_COLOR_WHITE,              /* Pure White   (255, 255, 255) */
    RGB_COLOR_WARM_WHITE,         /* Warm White   (255, 230, 180) */
    RGB_COLOR_YELLOW,             /* Yellow       (255, 255,   0) */
    RGB_COLOR_CYAN,               /* Cyan/Aqua    (  0, 255, 255) */
    RGB_COLOR_MAGENTA,            /* Magenta      (255,   0, 255) */
    RGB_COLOR_ORANGE,             /* Orange       (255, 120,   0) */
    RGB_COLOR_PURPLE,             /* Purple       (160,  32, 240) */
    RGB_COLOR_PINK,               /* Hot Pink     (255, 105, 180) */
    RGB_COLOR_AMBER,              /* Amber        (255, 191,   0) */
    RGB_COLOR_LIME,               /* Lime Green   ( 50, 205,  50) */
    RGB_COLOR_TEAL,               /* Teal         (  0, 128, 128) */
    RGB_COLOR_INDIGO,             /* Indigo       ( 75,   0, 130) */
    RGB_COLOR_GOLD,               /* Gold         (255, 215,   0) */
    RGB_COLOR_VIOLET,             /* Violet       (238, 130, 238) */
    RGB_COLOR_TURQUOISE,          /* Turquoise    ( 64, 224, 208) */
    RGB_COLOR_CUSTOM              /* Custom RGB via rgb_led_set_rgb */
} rgb_color_t;

/* RGB Color Triple (0 - 255 per channel) */
typedef struct {
    uint8 r;
    uint8 g;
    uint8 b;
} rgb_color_val_t;

/* Configuration Struct */
typedef struct {
    uint8          red_pin;       /* e.g., RGB_LED_PIN(B, 5) */
    uint8          green_pin;     /* e.g., RGB_LED_PIN(B, 6) */
    uint8          blue_pin;      /* e.g., RGB_LED_PIN(B, 7) */
    rgb_led_type_t type;          /* Common Anode or Cathode */
} rgb_led_cfg_t;

/* Driver Instance Handle */
typedef struct {
    GPIO_TypeDef   *red_port;
    uint16          red_pin;
    GPIO_TypeDef   *green_port;
    uint16          green_pin;
    GPIO_TypeDef   *blue_port;
    uint16          blue_pin;
    rgb_led_type_t  type;
    rgb_color_t     current_color;
    rgb_color_val_t target_val;
    rgb_color_val_t active_val;   /* Target scaled by brightness */
    uint8           brightness;   /* Brightness percentage: 0 - 100 % */
    uint8           pwm_step;     /* Software PWM counter: 0 - (RGB_LED_PWM_STEPS - 1) */
    bool            is_on;
} rgb_led_t;

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/**
 * @brief  Initialize the RGB LED driver instance with specified pins and connection type.
 * @param  led  Pointer to instance handle.
 * @param  cfg  Pointer to configuration structure.
 * @retval E_OK on success, E_NOT_OK on invalid parameters.
 */
Std_ReturnType rgb_led_init(rgb_led_t *led, const rgb_led_cfg_t *cfg);

/**
 * @brief  Set the RGB LED to one of the rich preset colors from rgb_color_t enum.
 * @param  led    Pointer to instance handle.
 * @param  color  Preset color from enum (e.g. RGB_COLOR_RED, RGB_COLOR_PURPLE).
 * @retval E_OK on success, E_NOT_OK on failure.
 */
Std_ReturnType rgb_led_set_color(rgb_led_t *led, rgb_color_t color);

/**
 * @brief  Set any custom RGB color by specifying 0-255 values for Red, Green, and Blue.
 * @param  led  Pointer to instance handle.
 * @param  r    Red intensity   (0 - 255)
 * @param  g    Green intensity (0 - 255)
 * @param  b    Blue intensity  (0 - 255)
 * @retval E_OK on success, E_NOT_OK on failure.
 */
Std_ReturnType rgb_led_set_rgb(rgb_led_t *led, uint8 r, uint8 g, uint8 b);

/**
 * @brief  Set the overall brightness percentage without changing the hue.
 * @param  led             Pointer to instance handle.
 * @param  brightness_pct  0 to 100 %
 * @retval E_OK on success, E_NOT_OK on failure.
 */
Std_ReturnType rgb_led_set_brightness(rgb_led_t *led, uint8 brightness_pct);

/**
 * @brief  Turn off the RGB LED completely.
 * @param  led  Pointer to instance handle.
 * @retval E_OK on success, E_NOT_OK on failure.
 */
Std_ReturnType rgb_led_off(rgb_led_t *led);

/**
 * @brief  Toggle the RGB LED (turns off if on, or restores previous color if off).
 * @param  led  Pointer to instance handle.
 * @retval E_OK on success, E_NOT_OK on failure.
 */
Std_ReturnType rgb_led_toggle(rgb_led_t *led);

/**
 * @brief  Get the current color preset of the RGB LED.
 * @param  led  Pointer to instance handle.
 * @retval Current rgb_color_t.
 */
rgb_color_t    rgb_led_get_color(const rgb_led_t *led);

/**
 * @brief  Get the current raw RGB component values (0-255).
 * @param  led  Pointer to instance handle.
 * @param  out  Pointer to output rgb_color_val_t structure.
 * @retval E_OK on success, E_NOT_OK on failure.
 */
Std_ReturnType rgb_led_get_rgb(const rgb_led_t *led, rgb_color_val_t *out);

/**
 * @brief  Cooperative service task for software PWM color blending.
 *         Call periodically from ecual_task() or SysTick.
 * @param  led  Pointer to instance handle.
 */
void           rgb_led_task(rgb_led_t *led);

#endif /* __RGB_LED_H */
