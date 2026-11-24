/******************* Section 0 : File Guard *******************/

/******************* Section 1 : Includes *******************/

#include "../../../Inc/ECUAL/Buzzer/buzzer.h"

/******************* Section 2 : Macros Declarations *******************/

#define BUZZER_PWM_BASE_FREQ_HZ       (1000000UL) /* 1 MHz timer tick counter */

/******************* Section 3 : Macros Functions Declarations **************/

/* (none) */

/************** Section 4 : Static / Private Variables & Presets **************/

/* Upbeat ascending chime (C6 -> E6): Item put into cart */
static const buzzer_note_t s_melody_item_added[] = {
    { NOTE_C6,  70U },
    { NOTE_REST, 30U },
    { NOTE_E6, 110U }
};

/* Descending chime (E6 -> C6): Item removed from cart */
static const buzzer_note_t s_melody_item_removed[] = {
    { NOTE_E6,  70U },
    { NOTE_REST, 30U },
    { NOTE_C6, 110U }
};

/* Dual low harsh warning buzz (A4 x 2): Weight mismatch / Scan error */
static const buzzer_note_t s_melody_cart_error[] = {
    { NOTE_A4, 130U },
    { NOTE_REST, 60U },
    { NOTE_A4, 200U }
};

/* Happy 4-tone victory arpeggio: Checkout / Payment successful */
static const buzzer_note_t s_melody_success[] = {
    { NOTE_C5,  80U },
    { NOTE_E5,  80U },
    { NOTE_G5,  80U },
    { NOTE_C6, 220U }
};

/* Warm power-on welcome chime: System Boot */
static const buzzer_note_t s_melody_boot[] = {
    { NOTE_C5, 100U },
    { NOTE_G5, 100U },
    { NOTE_C6, 160U }
};

/* Crisp ultra-short tick: Keypad / Button Click */
static const buzzer_note_t s_melody_click[] = {
    { NOTE_C7,  25U }
};

/************** Section 5 : Static Helper Functions **************/

static GPIO_TypeDef *buzzer_port_of(uint8 pin_code)
{
    uint8 port_idx = (pin_code >> 4U) & 0x0FU;
    switch (port_idx) {
        case BUZZER_PORT_A: return GPIOA;
        case BUZZER_PORT_B: return GPIOB;
        case BUZZER_PORT_C: return GPIOC;
        case BUZZER_PORT_D: return GPIOD;
        case BUZZER_PORT_E: return GPIOE;
        case BUZZER_PORT_H: return GPIOH;
        default:            return NULL;
    }
}

static uint16 buzzer_pin_of(uint8 pin_code)
{
    uint8 pin_num = pin_code & 0x0FU;
    return (uint16)(1U << pin_num);
}

static void buzzer_enable_gpio_clk(const GPIO_TypeDef *port)
{
    if (port == GPIOA)      { __HAL_RCC_GPIOA_CLK_ENABLE(); }
    else if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
    else if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
#if defined(GPIOD)
    else if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
#endif
#if defined(GPIOE)
    else if (port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
#endif
#if defined(GPIOH)
    else if (port == GPIOH) { __HAL_RCC_GPIOH_CLK_ENABLE(); }
#endif
}

static void buzzer_enable_tim_clk(const TIM_TypeDef *tim)
{
    if (tim == NULL) { return; }
#if defined(TIM1)
    if (tim == TIM1)        { __HAL_RCC_TIM1_CLK_ENABLE(); return; }
#endif
#if defined(TIM2)
    if (tim == TIM2)        { __HAL_RCC_TIM2_CLK_ENABLE(); return; }
#endif
#if defined(TIM3)
    if (tim == TIM3)        { __HAL_RCC_TIM3_CLK_ENABLE(); return; }
#endif
#if defined(TIM4)
    if (tim == TIM4)        { __HAL_RCC_TIM4_CLK_ENABLE(); return; }
#endif
#if defined(TIM5)
    if (tim == TIM5)        { __HAL_RCC_TIM5_CLK_ENABLE(); return; }
#endif
#if defined(TIM9)
    if (tim == TIM9)        { __HAL_RCC_TIM9_CLK_ENABLE(); return; }
#endif
#if defined(TIM10)
    if (tim == TIM10)       { __HAL_RCC_TIM10_CLK_ENABLE(); return; }
#endif
#if defined(TIM11)
    if (tim == TIM11)       { __HAL_RCC_TIM11_CLK_ENABLE(); return; }
#endif
}

static uint8 buzzer_get_tim_af(const TIM_TypeDef *tim)
{
    if (tim == NULL) { return 0U; }
#if defined(TIM1)
    if (tim == TIM1) { return GPIO_AF1_TIM1; }
#endif
#if defined(TIM2)
    if (tim == TIM2) { return GPIO_AF1_TIM2; }
#endif
#if defined(TIM3)
    if (tim == TIM3) { return GPIO_AF2_TIM3; }
#endif
#if defined(TIM4)
    if (tim == TIM4) { return GPIO_AF2_TIM4; }
#endif
#if defined(TIM5)
    if (tim == TIM5) { return GPIO_AF2_TIM5; }
#endif
#if defined(TIM9)
    if (tim == TIM9) { return GPIO_AF3_TIM9; }
#endif
#if defined(TIM10)
    if (tim == TIM10) { return GPIO_AF3_TIM10; }
#endif
#if defined(TIM11)
    if (tim == TIM11) { return GPIO_AF3_TIM11; }
#endif
    return GPIO_AF1_TIM2;
}

static void buzzer_pin_write(buzzer_t *bz, bool state)
{
    if (bz == NULL || bz->port == NULL) { return; }

    bool physical_high = bz->active_high ? state : !state;
    if (physical_high) {
        bz->port->BSRR = (uint32)bz->pin;
    } else {
        bz->port->BSRR = (uint32)bz->pin << 16U;
    }
}

static void buzzer_hw_pwm_set_freq(buzzer_t *bz, uint16 freq_hz)
{
    if (bz == NULL || bz->tim == NULL) { return; }

    if (freq_hz == 0U) {
        /* Rest / Mute: stop timer output */
        bz->tim->CR1 &= ~TIM_CR1_CEN;
        return;
    }

    /* Calculate timer period based on SystemCoreClock */
    uint32 pclk = SystemCoreClock;
    uint32 psc = (pclk / BUZZER_PWM_BASE_FREQ_HZ);
    if (psc > 0U) { psc -= 1U; }

    uint32 arr = (BUZZER_PWM_BASE_FREQ_HZ / (uint32)freq_hz);
    if (arr > 0U) { arr -= 1U; }

    bz->tim->PSC = (uint16)psc;
    bz->tim->ARR = arr;

    /* 50% Duty cycle */
    uint32 ccr = (arr + 1U) / 2U;
    switch (bz->tim_channel) {
        case 1U: bz->tim->CCR1 = ccr; break;
        case 2U: bz->tim->CCR2 = ccr; break;
        case 3U: bz->tim->CCR3 = ccr; break;
        case 4U: bz->tim->CCR4 = ccr; break;
        default: bz->tim->CCR1 = ccr; break;
    }

    /* Force register update and enable timer */
    bz->tim->EGR |= TIM_EGR_UG;
    bz->tim->CR1 |= TIM_CR1_CEN;
}

static void buzzer_apply_note(buzzer_t *bz, uint16 freq_hz)
{
    if (bz == NULL) { return; }

    if (bz->type == BUZZER_TYPE_ACTIVE) {
        /* Active buzzer requires pure DC HIGH/LOW */
        buzzer_pin_write(bz, (freq_hz > 0U));
    } else {
        /* Passive buzzer requires square wave frequency */
        if (bz->tim != NULL) {
            buzzer_hw_pwm_set_freq(bz, freq_hz);
        } else {
            buzzer_pin_write(bz, (freq_hz > 0U));
        }
    }
}

/*************** Section 6 : Software Interfaces Definitions (APIs) ***************/

Std_ReturnType buzzer_init(buzzer_t *bz, const buzzer_cfg_t *cfg)
{
    if (bz == NULL || cfg == NULL) {
        return E_NOT_OK;
    }

    GPIO_TypeDef *port = buzzer_port_of(cfg->pin);
    uint16 pin = buzzer_pin_of(cfg->pin);

    if (port == NULL || pin == 0U) {
        return E_NOT_OK;
    }

    bz->port             = port;
    bz->pin              = pin;
    bz->type             = cfg->type;
    bz->active_high      = cfg->active_high;
    bz->tim              = cfg->tim;
    bz->tim_channel      = (cfg->tim_channel > 0U && cfg->tim_channel <= 4U) ? cfg->tim_channel : 1U;
    bz->melody_len       = 0U;
    bz->current_note_idx = 0U;
    bz->note_start_tick  = 0U;
    bz->is_playing       = false;
    bz->is_beeping       = false;
    bz->beep_end_tick    = 0U;

    buzzer_enable_gpio_clk(port);

    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin   = pin;
    gpio_init.Pull  = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;

    if ((bz->type == BUZZER_TYPE_PASSIVE) && (bz->tim != NULL)) {
        /* Hardware PWM alternate function mode */
        buzzer_enable_tim_clk(bz->tim);
        gpio_init.Mode      = GPIO_MODE_AF_PP;
        gpio_init.Alternate = buzzer_get_tim_af(bz->tim);
        HAL_GPIO_Init(port, &gpio_init);

        /* Configure Channel PWM Mode 1 (OCxM = 110) + preload */
        switch (bz->tim_channel) {
            case 1U:
                bz->tim->CCMR1 |= (6U << 4U) | TIM_CCMR1_OC1PE;
                bz->tim->CCER  |= TIM_CCER_CC1E;
                break;
            case 2U:
                bz->tim->CCMR1 |= (6U << 12U) | TIM_CCMR1_OC2PE;
                bz->tim->CCER  |= TIM_CCER_CC2E;
                break;
            case 3U:
                bz->tim->CCMR2 |= (6U << 4U) | TIM_CCMR2_OC3PE;
                bz->tim->CCER  |= TIM_CCER_CC3E;
                break;
            case 4U:
                bz->tim->CCMR2 |= (6U << 12U) | TIM_CCMR2_OC4PE;
                bz->tim->CCER  |= TIM_CCER_CC4E;
                break;
            default:
                break;
        }
        bz->tim->CR1 |= TIM_CR1_ARPE;
        bz->tim->CR1 &= ~TIM_CR1_CEN; /* Start muted */
    } else {
        /* Standard GPIO output mode */
        gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(port, &gpio_init);
        buzzer_pin_write(bz, false);
    }

    return E_OK;
}

Std_ReturnType buzzer_beep(buzzer_t *bz, uint16 duration_ms)
{
    if (bz == NULL || duration_ms == 0U) {
        return E_NOT_OK;
    }

    bz->is_playing    = false;
    bz->is_beeping    = true;
    bz->beep_end_tick = HAL_GetTick() + (uint32)duration_ms;

    /* Play a pleasant 2.4 kHz beep (or DC High if active) */
    buzzer_apply_note(bz, 2400U);

    return E_OK;
}

Std_ReturnType buzzer_play_tone(buzzer_t *bz, uint16 freq_hz, uint16 duration_ms)
{
    if (bz == NULL || duration_ms == 0U) {
        return E_NOT_OK;
    }

    buzzer_note_t single_note;
    single_note.freq_hz     = freq_hz;
    single_note.duration_ms = duration_ms;

    return buzzer_play_melody(bz, &single_note, 1U);
}

Std_ReturnType buzzer_play_melody(buzzer_t *bz, const buzzer_note_t *notes, uint8 note_count)
{
    if (bz == NULL || notes == NULL || note_count == 0U) {
        return E_NOT_OK;
    }

    uint8 count = (note_count > BUZZER_MAX_MELODY_NOTES) ? BUZZER_MAX_MELODY_NOTES : note_count;
    for (uint8 i = 0U; i < count; i++) {
        bz->melody_queue[i] = notes[i];
    }

    bz->melody_len       = count;
    bz->current_note_idx = 0U;
    bz->note_start_tick  = HAL_GetTick();
    bz->is_beeping       = false;
    bz->is_playing       = true;

    /* Trigger the first note */
    buzzer_apply_note(bz, bz->melody_queue[0].freq_hz);

    return E_OK;
}

Std_ReturnType buzzer_play_preset(buzzer_t *bz, buzzer_preset_t preset)
{
    if (bz == NULL) {
        return E_NOT_OK;
    }

    switch (preset) {
        case BUZZER_MELODY_ITEM_ADDED:
            return buzzer_play_melody(bz, s_melody_item_added,
                                      sizeof(s_melody_item_added) / sizeof(s_melody_item_added[0]));

        case BUZZER_MELODY_ITEM_REMOVED:
            return buzzer_play_melody(bz, s_melody_item_removed,
                                      sizeof(s_melody_item_removed) / sizeof(s_melody_item_removed[0]));

        case BUZZER_MELODY_CART_ERROR:
            return buzzer_play_melody(bz, s_melody_cart_error,
                                      sizeof(s_melody_cart_error) / sizeof(s_melody_cart_error[0]));

        case BUZZER_MELODY_SUCCESS:
            return buzzer_play_melody(bz, s_melody_success,
                                      sizeof(s_melody_success) / sizeof(s_melody_success[0]));

        case BUZZER_MELODY_BOOT:
            return buzzer_play_melody(bz, s_melody_boot,
                                      sizeof(s_melody_boot) / sizeof(s_melody_boot[0]));

        case BUZZER_MELODY_CLICK:
            return buzzer_play_melody(bz, s_melody_click,
                                      sizeof(s_melody_click) / sizeof(s_melody_click[0]));

        default:
            buzzer_stop(bz);
            return E_OK;
    }
}

Std_ReturnType buzzer_stop(buzzer_t *bz)
{
    if (bz == NULL) {
        return E_NOT_OK;
    }

    bz->is_playing    = false;
    bz->is_beeping    = false;
    bz->melody_len    = 0U;

    buzzer_apply_note(bz, 0U);

    return E_OK;
}

bool buzzer_is_busy(const buzzer_t *bz)
{
    if (bz == NULL) {
        return false;
    }
    return (bz->is_playing || bz->is_beeping);
}

void buzzer_task(buzzer_t *bz)
{
    if (bz == NULL) {
        return;
    }

    uint32 current_tick = HAL_GetTick();

    /* Handle simple duration beep */
    if (bz->is_beeping) {
        if (current_tick >= bz->beep_end_tick) {
            buzzer_stop(bz);
        }
        return;
    }

    /* Handle melody sequencer */
    if (bz->is_playing) {
        uint32 elapsed = current_tick - bz->note_start_tick;
        if (elapsed >= (uint32)bz->melody_queue[bz->current_note_idx].duration_ms) {
            bz->current_note_idx++;

            if (bz->current_note_idx >= bz->melody_len) {
                /* Finished playing all notes */
                buzzer_stop(bz);
            } else {
                /* Play next note */
                bz->note_start_tick = current_tick;
                buzzer_apply_note(bz, bz->melody_queue[bz->current_note_idx].freq_hz);
            }
        }
    }
}
