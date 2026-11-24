/******************* Section 0 : File Guard *******************/

/******************* Section 1 : Includes *******************/

#include "../../../Inc/ECUAL/Button/button.h"

/******************* Section 2 : Macros Declarations *******************/

/* (none) */

/******************* Section 3 : Macros Functions Declarations **************/

/* (none) */

/************** Section 4 : Static / Private Variables Declarations **************/

/* Head of linked list for all registered button instances */
static button_t *s_button_head = NULL;

/************** Section 5 : Static Helper Functions **************/

static GPIO_TypeDef *button_port_of(uint8 pin_code)
{
    uint8 port_idx = (pin_code >> 4U) & 0x0FU;
    switch (port_idx) {
        case BUTTON_PORT_A: return GPIOA;
        case BUTTON_PORT_B: return GPIOB;
        case BUTTON_PORT_C: return GPIOC;
        case BUTTON_PORT_D: return GPIOD;
        case BUTTON_PORT_E: return GPIOE;
        case BUTTON_PORT_H: return GPIOH;
        default:            return NULL;
    }
}

static uint8 button_pin_num_of(uint8 pin_code)
{
    return (pin_code & 0x0FU);
}

static uint16 button_pin_of(uint8 pin_code)
{
    return (uint16)(1U << button_pin_num_of(pin_code));
}

static void button_enable_gpio_clk(const GPIO_TypeDef *port)
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

static IRQn_Type button_get_exti_irq(uint8 pin_num)
{
    switch (pin_num) {
        case 0U:  return EXTI0_IRQn;
        case 1U:  return EXTI1_IRQn;
        case 2U:  return EXTI2_IRQn;
        case 3U:  return EXTI3_IRQn;
        case 4U:  return EXTI4_IRQn;
        case 5U:
        case 6U:
        case 7U:
        case 8U:
        case 9U:  return EXTI9_5_IRQn;
        case 10U:
        case 11U:
        case 12U:
        case 13U:
        case 14U:
        case 15U: return EXTI15_10_IRQn;
        default:  return EXTI0_IRQn;
    }
}

static button_state_t button_read_raw(const button_t *btn)
{
    if (btn == NULL || btn->port == NULL) {
        return BUTTON_STATE_RELEASED;
    }

    uint8 pin_level = ((btn->port->IDR & btn->pin) != 0U) ? 1U : 0U;

    if (btn->active_level == BUTTON_ACTIVE_LOW) {
        /* Pressed when pin is LOW (GND) */
        return (pin_level == 0U) ? BUTTON_STATE_PRESSED : BUTTON_STATE_RELEASED;
    } else {
        /* Pressed when pin is HIGH (VCC) */
        return (pin_level != 0U) ? BUTTON_STATE_PRESSED : BUTTON_STATE_RELEASED;
    }
}

static void button_register_instance(button_t *btn)
{
    if (btn == NULL) { return; }

    /* Check if already in list */
    button_t *curr = s_button_head;
    while (curr != NULL) {
        if (curr == btn) { return; }
        curr = curr->next;
    }

    btn->next = s_button_head;
    s_button_head = btn;
}

/*************** Section 6 : Software Interfaces Definitions (APIs) ***************/

Std_ReturnType button_init(button_t *btn, const button_cfg_t *cfg)
{
    if (btn == NULL || cfg == NULL) {
        return E_NOT_OK;
    }

    GPIO_TypeDef *port = button_port_of(cfg->pin);
    uint8 pin_num = button_pin_num_of(cfg->pin);
    uint16 pin = button_pin_of(cfg->pin);

    if (port == NULL || pin == 0U) {
        return E_NOT_OK;
    }

    btn->port                  = port;
    btn->pin                   = pin;
    btn->pin_num               = pin_num;
    btn->active_level          = cfg->active_level;
    btn->mode                  = cfg->mode;
    btn->debounce_ms           = (cfg->debounce_ms > 0U) ? cfg->debounce_ms : BUTTON_DEFAULT_DEBOUNCE_MS;
    btn->long_press_ms         = (cfg->long_press_ms > 0U) ? cfg->long_press_ms : BUTTON_DEFAULT_LONG_PRESS_MS;
    btn->double_click_ms       = (cfg->double_click_ms > 0U) ? cfg->double_click_ms : BUTTON_DEFAULT_DOUBLE_CLICK_MS;
    btn->callback              = cfg->callback;
    btn->user_data             = cfg->user_data;

    btn->last_debounce_tick    = 0U;
    btn->press_start_tick      = 0U;
    btn->last_release_tick     = 0U;
    btn->click_count           = 0U;
    btn->long_press_fired      = false;
    btn->exti_flag             = false;
    btn->next                  = NULL;

    button_enable_gpio_clk(port);

    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin  = pin;
    gpio_init.Pull = (uint32)cfg->pull;

    if (cfg->mode == BUTTON_MODE_EXTI) {
        __HAL_RCC_SYSCFG_CLK_ENABLE();
        /* Trigger interrupt on both edges to accurately detect Press and Release */
        gpio_init.Mode = GPIO_MODE_IT_RISING_FALLING;
        HAL_GPIO_Init(port, &gpio_init);

        IRQn_Type irq_num = button_get_exti_irq(pin_num);
        HAL_NVIC_SetPriority(irq_num, cfg->exti_preempt_priority, cfg->exti_sub_priority);
        HAL_NVIC_EnableIRQ(irq_num);
    } else {
        gpio_init.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init(port, &gpio_init);
    }

    /* Initialize stable state */
    btn->current_state  = button_read_raw(btn);
    btn->last_raw_state = btn->current_state;

    button_register_instance(btn);

    return E_OK;
}

button_state_t button_get_state(const button_t *btn)
{
    if (btn == NULL) {
        return BUTTON_STATE_RELEASED;
    }
    return btn->current_state;
}

bool button_is_pressed(const button_t *btn)
{
    return (button_get_state(btn) == BUTTON_STATE_PRESSED);
}

void button_set_callback(button_t *btn, button_callback_t cb, void *user_data)
{
    if (btn == NULL) { return; }
    btn->callback  = cb;
    btn->user_data = user_data;
}

void button_task(button_t *btn)
{
    if (btn == NULL) { return; }

    uint32 current_tick = HAL_GetTick();
    button_state_t raw_state = button_read_raw(btn);

    /* 1. Instant Edge Trigger with Lockout Debounce:
     * Responds with ZERO latency on the very first contact,
     * then locks out bouncing for debounce_ms.
     */
    if ((raw_state != btn->current_state) &&
        ((current_tick - btn->last_debounce_tick) >= (uint32)btn->debounce_ms)) {

        btn->current_state      = raw_state;
        btn->last_debounce_tick = current_tick;

        if (btn->current_state == BUTTON_STATE_PRESSED) {
            /* Button transition: RELEASED -> PRESSED */
            btn->press_start_tick = current_tick;
            btn->long_press_fired = false;

            if (btn->callback != NULL) {
                btn->callback(btn, BUTTON_EVENT_PRESSED, btn->user_data);
            }
        } else {
            /* Button transition: PRESSED -> RELEASED */
            if (btn->callback != NULL) {
                btn->callback(btn, BUTTON_EVENT_RELEASED, btn->user_data);
            }

            if (!btn->long_press_fired) {
                btn->click_count++;
                btn->last_release_tick = current_tick;
            }
        }
    }

    /* 2. Long Press Detection (Evaluated continuously while held down) */
    if (btn->current_state == BUTTON_STATE_PRESSED && !btn->long_press_fired) {
        if ((current_tick - btn->press_start_tick) >= (uint32)btn->long_press_ms) {
            btn->long_press_fired = true;
            btn->click_count      = 0U;

            if (btn->callback != NULL) {
                btn->callback(btn, BUTTON_EVENT_LONG_PRESS, btn->user_data);
            }
        }
    }

    /* 3. Single Click / Double Click Evaluation (Evaluated after release) */
    if (btn->current_state == BUTTON_STATE_RELEASED && btn->click_count > 0U) {
        if (btn->click_count >= 2U) {
            /* Double click detected! */
            btn->click_count = 0U;
            if (btn->callback != NULL) {
                btn->callback(btn, BUTTON_EVENT_DOUBLE_CLICK, btn->user_data);
            }
        } else if ((current_tick - btn->last_release_tick) >= (uint32)btn->double_click_ms) {
            /* Single click timeout window elapsed */
            btn->click_count = 0U;
            if (btn->callback != NULL) {
                btn->callback(btn, BUTTON_EVENT_CLICK, btn->user_data);
            }
        }
    }
}

void button_task_all(void)
{
    button_t *curr = s_button_head;
    while (curr != NULL) {
        button_task(curr);
        curr = curr->next;
    }
}

void button_exti_handler(uint16 gpio_pin)
{
    button_t *curr = s_button_head;
    while (curr != NULL) {
        if (curr->pin == gpio_pin) {
            curr->exti_flag = true;
            break;
        }
        curr = curr->next;
    }
}

/*************** Section 7 : EXTI Interrupt Handlers & Callbacks ***************/

/* Forward HAL EXTI callback to button_exti_handler */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    button_exti_handler(GPIO_Pin);
}

/* Weak default EXTI ISR definitions overriding startup file weak symbols */
__attribute__((weak)) void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

__attribute__((weak)) void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

__attribute__((weak)) void EXTI2_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

__attribute__((weak)) void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

__attribute__((weak)) void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

__attribute__((weak)) void EXTI9_5_IRQHandler(void)
{
    for (uint16_t pin = GPIO_PIN_5; pin <= GPIO_PIN_9; pin <<= 1U) {
        if (__HAL_GPIO_EXTI_GET_IT(pin) != RESET) {
            HAL_GPIO_EXTI_IRQHandler(pin);
        }
    }
}

__attribute__((weak)) void EXTI15_10_IRQHandler(void)
{
    for (uint32_t pin = GPIO_PIN_10; pin <= GPIO_PIN_15; pin <<= 1U) {
        if (__HAL_GPIO_EXTI_GET_IT((uint16_t)pin) != RESET) {
            HAL_GPIO_EXTI_IRQHandler((uint16_t)pin);
        }
    }
}
