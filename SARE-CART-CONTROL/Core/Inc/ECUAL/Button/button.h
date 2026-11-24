/******************* Section 0 : File Guard *******************/

#ifndef __BUTTON_H
#define __BUTTON_H

/******************* Section 1 : Includes *******************/

#include "../../MCAL/mcal_std_types.h"

/******************* Section 2 : Macros Declarations *******************/

/* --- Driver Version --- */
#define BUTTON_VERSION_MAJOR          (1U)
#define BUTTON_VERSION_MINOR          (0U)
#define BUTTON_VERSION_PATCH          (0U)

/* --- Pin Encoding Macro: BUTTON_PIN(port, pin_number) ---
 * Example: BUTTON_PIN(A, 0) -> PA0, BUTTON_PIN(C, 13) -> PC13
 */
#define BUTTON_PORT_A                 (0U)
#define BUTTON_PORT_B                 (1U)
#define BUTTON_PORT_C                 (2U)
#define BUTTON_PORT_D                 (3U)
#define BUTTON_PORT_E                 (4U)
#define BUTTON_PORT_H                 (7U)
#define BUTTON_PIN(port, num)         ((uint8)(((uint8)BUTTON_PORT_##port << 4U) | ((uint8)(num) & 0x0FU)))

/* --- Default Timing Thresholds (ms) --- */
#define BUTTON_DEFAULT_DEBOUNCE_MS    (20U)
#define BUTTON_DEFAULT_LONG_PRESS_MS  (1000U)
#define BUTTON_DEFAULT_DOUBLE_CLICK_MS (250U)

/******************* Section 3 : Macros Functions Declarations **************/

/* (none) */

/************** Section 4 : User Defined Data Types & Variables Declarations **************/

/* Active Level (Logic state when physically pressed) */
typedef enum {
    BUTTON_ACTIVE_LOW  = 0,       /* Pin connects to GND on press (Pull-Up active) */
    BUTTON_ACTIVE_HIGH = 1        /* Pin connects to VCC on press (Pull-Down active) */
} button_active_level_t;

/* Internal Pull Resistor Configuration */
typedef enum {
    BUTTON_PULL_NONE = GPIO_NOPULL,
    BUTTON_PULL_UP   = GPIO_PULLUP,
    BUTTON_PULL_DOWN = GPIO_PULLDOWN
} button_pull_t;

/* Button Operation Mode */
typedef enum {
    BUTTON_MODE_POLLING = 0,      /* Periodic software sampling in button_task */
    BUTTON_MODE_EXTI    = 1       /* Hardware EXTI edge interrupts */
} button_mode_t;

/* Physical Debounced Button State */
typedef enum {
    BUTTON_STATE_RELEASED = 0,
    BUTTON_STATE_PRESSED  = 1
} button_state_t;

/* Button Event Types Dispatched to Callbacks */
typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_PRESSED,         /* Instantaneous transition to PRESSED */
    BUTTON_EVENT_RELEASED,        /* Instantaneous transition to RELEASED */
    BUTTON_EVENT_CLICK,           /* Short click completed (Pressed & Released < long_press_ms) */
    BUTTON_EVENT_LONG_PRESS,      /* Held continuously for >= long_press_ms */
    BUTTON_EVENT_DOUBLE_CLICK     /* Two quick clicks within double_click_ms */
} button_event_t;

/* Forward declaration */
struct button_s;

/* Event Callback Signature */
typedef void (*button_callback_t)(struct button_s *btn, button_event_t event, void *user_data);

/* Configuration Struct */
typedef struct {
    uint8                 pin;                  /* BUTTON_PIN(port, pin) */
    button_active_level_t active_level;         /* ACTIVE_LOW or ACTIVE_HIGH */
    button_pull_t         pull;                 /* PULL_UP, PULL_DOWN, or NONE */
    button_mode_t         mode;                 /* POLLING or EXTI */
    uint8                 exti_preempt_priority;/* Preemption priority (0-15) if EXTI */
    uint8                 exti_sub_priority;    /* Subpriority (0-15) if EXTI */
    uint16                debounce_ms;          /* Debounce filter time in ms (0 = default 40ms) */
    uint16                long_press_ms;        /* Long-press threshold in ms (0 = default 1000ms) */
    uint16                double_click_ms;      /* Double-click window in ms (0 = default 300ms) */
    button_callback_t     callback;             /* Optional callback function pointer */
    void                 *user_data;            /* Optional user context pointer */
} button_cfg_t;

/* Driver Instance Handle */
typedef struct button_s {
    GPIO_TypeDef         *port;
    uint16                pin;
    uint8                 pin_num;
    button_active_level_t active_level;
    button_mode_t         mode;
    uint16                debounce_ms;
    uint16                long_press_ms;
    uint16                double_click_ms;
    button_callback_t     callback;
    void                 *user_data;

    /* Internal State Tracking */
    button_state_t        current_state;        /* Stable debounced state */
    button_state_t        last_raw_state;       /* Last immediate sampled state */
    uint32                last_debounce_tick;   /* Tick of last state change */
    uint32                press_start_tick;     /* Tick when button was pressed */
    uint32                last_release_tick;    /* Tick when button was last released */
    uint8                 click_count;          /* Multi-click accumulator */
    bool                  long_press_fired;     /* Flag to prevent duplicate long press events */
    volatile bool         exti_flag;            /* Set by EXTI ISR, cleared in task */

    struct button_s      *next;                 /* Linked list pointer for registry */
} button_t;

/*************** Section 5 : Software Interfaces Declarations (APIs) ***************/

/**
 * @brief  Initialize a push button instance with pin, pull resistor, active level, and mode.
 * @param  btn  Pointer to button instance.
 * @param  cfg  Pointer to button configuration.
 * @retval E_OK on success, E_NOT_OK on invalid parameters.
 */
Std_ReturnType button_init(button_t *btn, const button_cfg_t *cfg);

/**
 * @brief  Get the current debounced physical state of the button (PRESSED or RELEASED).
 * @param  btn  Pointer to button instance.
 * @retval BUTTON_STATE_PRESSED or BUTTON_STATE_RELEASED.
 */
button_state_t button_get_state(const button_t *btn);

/**
 * @brief  Convenience function to check if the button is currently held down.
 * @param  btn  Pointer to button instance.
 * @retval true if pressed, false if released.
 */
bool           button_is_pressed(const button_t *btn);

/**
 * @brief  Register or update the event callback for a button instance.
 * @param  btn        Pointer to button instance.
 * @param  cb         Callback function pointer.
 * @param  user_data  User data pointer passed to callback.
 */
void           button_set_callback(button_t *btn, button_callback_t cb, void *user_data);

/**
 * @brief  Periodic button task to execute debouncing, timing, and event dispatching.
 *         Must be called periodically in the main loop or ecual_task().
 * @param  btn  Pointer to button instance.
 */
void           button_task(button_t *btn);

/**
 * @brief  Convenience function to service all initialized button instances at once.
 */
void           button_task_all(void);

/**
 * @brief  EXTI Interrupt Service Handler. Called from HAL_GPIO_EXTI_Callback or EXTI IRQs.
 * @param  gpio_pin  GPIO pin mask that triggered the interrupt.
 */
void           button_exti_handler(uint16 gpio_pin);

#endif /* __BUTTON_H */
