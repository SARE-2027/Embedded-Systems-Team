/******************* Section 0 : File Header *******************/

/**
 *  rgb_led.c  -  Universal RGB LED Strip Driver (STM32F401RCTx)
 *
 *  Features:
 *    - Supports both Common Anode and Common Cathode RGB LED strips.
 *    - Configurable to any 3 GPIO pins on any port.
 *    - Rich preset color palette with one-call color selection.
 *    - Full 24-bit custom RGB (0-255 per channel) with software PWM blending.
 *    - Independent brightness scaling (0 - 100%).
 *    - Direct atomic register pin control (BSRR).
 */

/******************* Section 1 : Includes *******************/

#include "../../../Inc/ECUAL/RGB_LED/rgb_led.h"

/******************* Section 2 : Macros Declarations *******************/

#define RGB_DEFAULT_BRIGHTNESS        (100U)

/******************* Section 3 : Variables Definitions *******************/

/* Preset Color Table: maps rgb_color_t to standard (R, G, B) triples */
static const rgb_color_val_t m_color_palette[] = {
    [RGB_COLOR_OFF]        = {   0U,   0U,   0U },
    [RGB_COLOR_RED]        = { 255U,   0U,   0U },
    [RGB_COLOR_GREEN]      = {   0U, 255U,   0U },
    [RGB_COLOR_BLUE]       = {   0U,   0U, 255U },
    [RGB_COLOR_WHITE]      = { 255U, 255U, 255U },
    [RGB_COLOR_WARM_WHITE] = { 255U, 230U, 180U },
    [RGB_COLOR_YELLOW]     = { 255U, 255U,   0U },
    [RGB_COLOR_CYAN]       = {   0U, 255U, 255U },
    [RGB_COLOR_MAGENTA]    = { 255U,   0U, 255U },
    [RGB_COLOR_ORANGE]     = { 255U, 120U,   0U },
    [RGB_COLOR_PURPLE]     = { 160U,  32U, 240U },
    [RGB_COLOR_PINK]       = { 255U, 105U, 180U },
    [RGB_COLOR_AMBER]      = { 255U, 191U,   0U },
    [RGB_COLOR_LIME]       = {  50U, 205U,  50U },
    [RGB_COLOR_TEAL]       = {   0U, 128U, 128U },
    [RGB_COLOR_INDIGO]     = {  75U,   0U, 130U },
    [RGB_COLOR_GOLD]       = { 255U, 215U,   0U },
    [RGB_COLOR_VIOLET]     = { 238U, 130U, 238U },
    [RGB_COLOR_TURQUOISE]  = {  64U, 224U, 208U }
};

#define PALETTE_SIZE (sizeof(m_color_palette) / sizeof(m_color_palette[0]))

/************** Section 4 : Helper Functions Declarations **************/

static GPIO_TypeDef *rgb_led_port_of(uint8 pin_id);
static void          rgb_led_gpio_clk_enable(const GPIO_TypeDef *port);
static void          rgb_led_apply_static(rgb_led_t *led);
static void          rgb_led_update_active_val(rgb_led_t *led);
static void          rgb_led_write_channel(GPIO_TypeDef *port, uint16 pin, rgb_led_type_t type, bool state);

/*************** Section 5 : Software Interfaces Definitions (APIs) ****************/

Std_ReturnType rgb_led_init(rgb_led_t *led, const rgb_led_cfg_t *cfg)
{
    GPIO_InitTypeDef gpio = {0};

    if ((NULL == led) || (NULL == cfg)) {
        return E_NOT_OK;
    }

    led->red_port   = rgb_led_port_of(cfg->red_pin);
    led->red_pin    = (uint16)(1U << (cfg->red_pin & 0x0FU));
    led->green_port = rgb_led_port_of(cfg->green_pin);
    led->green_pin  = (uint16)(1U << (cfg->green_pin & 0x0FU));
    led->blue_port  = rgb_led_port_of(cfg->blue_pin);
    led->blue_pin   = (uint16)(1U << (cfg->blue_pin & 0x0FU));

    if ((NULL == led->red_port) || (NULL == led->green_port) || (NULL == led->blue_port)) {
        return E_NOT_OK;
    }

    led->type          = cfg->type;
    led->brightness    = RGB_DEFAULT_BRIGHTNESS;
    led->current_color = RGB_COLOR_OFF;
    led->target_val    = (rgb_color_val_t){0U, 0U, 0U};
    led->active_val    = (rgb_color_val_t){0U, 0U, 0U};
    led->pwm_step      = 0U;
    led->is_on         = false;

    /* Initialize Red pin */
    rgb_led_gpio_clk_enable(led->red_port);
    gpio.Pin   = led->red_pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(led->red_port, &gpio);

    /* Initialize Green pin */
    rgb_led_gpio_clk_enable(led->green_port);
    gpio.Pin   = led->green_pin;
    HAL_GPIO_Init(led->green_port, &gpio);

    /* Initialize Blue pin */
    rgb_led_gpio_clk_enable(led->blue_port);
    gpio.Pin   = led->blue_pin;
    HAL_GPIO_Init(led->blue_port, &gpio);

    /* Set default state: all OFF */
    (void)rgb_led_off(led);

    return E_OK;
}

Std_ReturnType rgb_led_set_color(rgb_led_t *led, rgb_color_t color)
{
    if ((NULL == led) || ((uint8)color >= PALETTE_SIZE)) {
        return E_NOT_OK;
    }

    led->current_color = color;
    led->target_val    = m_color_palette[(uint8)color];
    led->is_on         = (color != RGB_COLOR_OFF);

    rgb_led_update_active_val(led);
    rgb_led_apply_static(led);

    return E_OK;
}

Std_ReturnType rgb_led_set_rgb(rgb_led_t *led, uint8 r, uint8 g, uint8 b)
{
    if (NULL == led) {
        return E_NOT_OK;
    }

    led->current_color = RGB_COLOR_CUSTOM;
    led->target_val    = (rgb_color_val_t){ r, g, b };
    led->is_on         = ((r > 0U) || (g > 0U) || (b > 0U));

    rgb_led_update_active_val(led);
    rgb_led_apply_static(led);

    return E_OK;
}

Std_ReturnType rgb_led_set_brightness(rgb_led_t *led, uint8 brightness_pct)
{
    if (NULL == led) {
        return E_NOT_OK;
    }

    if (brightness_pct > 100U) {
        brightness_pct = 100U;
    }

    led->brightness = brightness_pct;
    rgb_led_update_active_val(led);
    rgb_led_apply_static(led);

    return E_OK;
}

Std_ReturnType rgb_led_off(rgb_led_t *led)
{
    if (NULL == led) {
        return E_NOT_OK;
    }

    led->current_color = RGB_COLOR_OFF;
    led->active_val    = (rgb_color_val_t){ 0U, 0U, 0U };
    led->is_on         = false;

    rgb_led_apply_static(led);

    return E_OK;
}

Std_ReturnType rgb_led_toggle(rgb_led_t *led)
{
    if (NULL == led) {
        return E_NOT_OK;
    }

    if (led->is_on) {
        return rgb_led_off(led);
    } else {
        if (led->current_color == RGB_COLOR_OFF) {
            return rgb_led_set_color(led, RGB_COLOR_WHITE);
        } else {
            led->is_on = true;
            rgb_led_update_active_val(led);
            rgb_led_apply_static(led);
            return E_OK;
        }
    }
}

rgb_color_t rgb_led_get_color(const rgb_led_t *led)
{
    if (NULL == led) {
        return RGB_COLOR_OFF;
    }
    return led->current_color;
}

Std_ReturnType rgb_led_get_rgb(const rgb_led_t *led, rgb_color_val_t *out)
{
    if ((NULL == led) || (NULL == out)) {
        return E_NOT_OK;
    }
    *out = led->target_val;
    return E_OK;
}

void rgb_led_task(rgb_led_t *led)
{
    uint8 r_steps;
    uint8 g_steps;
    uint8 b_steps;

    if ((NULL == led) || !led->is_on) {
        return;
    }

    /* Convert 0-255 active values to 0-(RGB_LED_PWM_STEPS) threshold */
    r_steps = (uint8)(((uint16)led->active_val.r * (uint16)RGB_LED_PWM_STEPS) >> 8U);
    g_steps = (uint8)(((uint16)led->active_val.g * (uint16)RGB_LED_PWM_STEPS) >> 8U);
    b_steps = (uint8)(((uint16)led->active_val.b * (uint16)RGB_LED_PWM_STEPS) >> 8U);

    /* If purely static digital state (all 0 or max), skip software PWM to save cycles */
    if (((r_steps == 0U) || (r_steps >= (RGB_LED_PWM_STEPS - 1U))) &&
        ((g_steps == 0U) || (g_steps >= (RGB_LED_PWM_STEPS - 1U))) &&
        ((b_steps == 0U) || (b_steps >= (RGB_LED_PWM_STEPS - 1U)))) {
        return;
    }

    /* Software PWM step */
    led->pwm_step = (uint8)((led->pwm_step + 1U) % RGB_LED_PWM_STEPS);

    rgb_led_write_channel(led->red_port,   led->red_pin,   led->type, (led->pwm_step < r_steps));
    rgb_led_write_channel(led->green_port, led->green_pin, led->type, (led->pwm_step < g_steps));
    rgb_led_write_channel(led->blue_port,  led->blue_pin,  led->type, (led->pwm_step < b_steps));
}

/************** Section 6 : Helper Functions Definitions **************/

static GPIO_TypeDef *rgb_led_port_of(uint8 pin_id)
{
    switch (pin_id >> 4U) {
        case RGB_LED_PORT_A: return GPIOA;
        case RGB_LED_PORT_B: return GPIOB;
        case RGB_LED_PORT_C: return GPIOC;
        case RGB_LED_PORT_D: return GPIOD;
        case RGB_LED_PORT_E: return GPIOE;
        case RGB_LED_PORT_H: return GPIOH;
        default:             return NULL;
    }
}

static void rgb_led_gpio_clk_enable(const GPIO_TypeDef *port)
{
    if      (port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
    else if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
    else if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
    else if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
    else if (port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
    else if (port == GPIOH) { __HAL_RCC_GPIOH_CLK_ENABLE(); }
    else                    { /* unknown port */ }
}

static void rgb_led_update_active_val(rgb_led_t *led)
{
    if (!led->is_on) {
        led->active_val = (rgb_color_val_t){ 0U, 0U, 0U };
        return;
    }

    led->active_val.r = (uint8)(((uint16)led->target_val.r * (uint16)led->brightness) / 100U);
    led->active_val.g = (uint8)(((uint16)led->target_val.g * (uint16)led->brightness) / 100U);
    led->active_val.b = (uint8)(((uint16)led->target_val.b * (uint16)led->brightness) / 100U);
}

static void rgb_led_write_channel(GPIO_TypeDef *port, uint16 pin, rgb_led_type_t type, bool state)
{
    /* Common Cathode: High = ON, Low = OFF.
     * Common Anode  : Low = ON, High = OFF.
     */
    bool pin_high;

    if (type == RGB_LED_COMMON_ANODE) {
        pin_high = !state;
    } else {
        pin_high = state;
    }

    if (pin_high) {
        port->BSRR = (uint32)pin;          /* Set pin HIGH */
    } else {
        port->BSRR = ((uint32)pin) << 16U; /* Set pin LOW  */
    }
}

static void rgb_led_apply_static(rgb_led_t *led)
{
    rgb_led_write_channel(led->red_port,   led->red_pin,   led->type, (led->active_val.r > 127U));
    rgb_led_write_channel(led->green_port, led->green_pin, led->type, (led->active_val.g > 127U));
    rgb_led_write_channel(led->blue_port,  led->blue_pin,  led->type, (led->active_val.b > 127U));
}
