/******************* Section 0 : File Header *******************/

/**
 *  ecual_sim7600_port.c  -  SIM7600E-H Portable UART Core (STM32F401RCTx)
 *
 *  A fully-customizable, register-level UART core for the SIM7600 GPS driver.
 *  Any USART channel (USART1 / USART2 / USART6) can be bound to any factory
 *  alternate-function TX/RX pin pair at RUNTIME through sim7600_port_config_t.
 *
 *  Design:
 *    - TX: blocking, poll-based (TXE + TC), with per-call deadline.
 *    - RX: interrupt-driven into a caller-owned ring buffer (no DMA dependency,
 *          keeps the core 100% portable across any USART/pin combination).
 *    - Clock/BRR: computed from the LIVE RCC registers (HSI/HSE/PLL aware),
 *          so no hardcoded frequency assumptions.
 *    - Validation: pins are checked against the DS9716 Rev 11 mapping for the
 *          STM32F401RC LQFP64 package; cross-instance pin ownership is tracked.
 *
 *  Reference: RM0368 (USART chapter 20), DS9716 Rev 11 (pin/AF tables).
 */

/******************* Section 1 : Includes *******************/

#include "../../../Inc/ECUAL/SIM7600EH/sim7600_port.h"

#include <string.h>


/******************* Section 2 : Macros Declarations *******************/

/* --- CR2 STOP[13:12] encodings (CMSIS has no named STOP constants) --- */
/*   00 = 1 stop, 01 = 0.5, 10 = 2, 11 = 1.5  (RM0368)                        */
#define PORT_USART_CR2_STOP_1_5 ((uint32)3)
#define PORT_USART_CR2_STOP_2 ((uint32)2)

/* --- GPIO register field values (PORT_-prefixed to avoid HAL name clashes) ---
 */
#define PORT_GPIO_MODE_INPUT (0x0UL)
#define PORT_GPIO_MODE_OUTPUT (0x1UL)
#define PORT_GPIO_MODE_AF (0x2UL)
#define PORT_GPIO_MODE_ANALOG (0x3UL)
#define PORT_GPIO_SPEED_HIGH (0x3UL)      /* OSPEEDR field value     */
#define PORT_GPIO_PULL_NONE (0x0UL)       /* PUPDR field value       */
#define PORT_GPIO_PULL_UP (0x1UL)         /* PUPDR field value       */
#define PORT_GPIO_OTYPE_PUSH_PULL (0x0UL) /* OTYPER bit value        */

/* --- GPIO register access via port nibble encoding (GPIOA + 0x400 * n) --- */
#define GPIO_PORT_STRIDE (0x400UL)
#define GPIO_PIN_PORT(pin) (((uint32)(pin)) >> 4) /* 0=PA..3=PD */
#define GPIO_PIN_INDEX(pin) (((uint32)(pin)) & 0x0FU)

/* --- Port index bounds for the ownership tracker --- */
#define GPIO_PORT_MAX (4U) /* A, B, C, D                */
#define GPIO_PIN_MAX (16U)

/******************* Section 3 : Macros Functions Declarations **************/

/* --- IRQ-safe critical section (preserves caller context) --- */
#define PORT_ENTER_CRITICAL(prim)                                              \
  do {                                                                         \
    (prim) = __get_PRIMASK();                                                  \
    __disable_irq();                                                           \
  } while (0)
#define PORT_EXIT_CRITICAL(prim)                                               \
  do {                                                                         \
    __set_PRIMASK((prim));                                                     \
  } while (0)

/************** Section 4 : User Defined Data Types & Variables Declarations
 * **************/

/* --- Factory AF mapping row: a pin and the alternate function number --- */
typedef struct {
  sim7600_pin_t pin;
  uint8 af;
} sim7600_port_pin_map_t;

/* --- IRQ-related instance parameters --- */
typedef enum {
  PORT_IRQ_USART1 = 0,
  PORT_IRQ_USART2,
  PORT_IRQ_USART6,
  PORT_IRQ_COUNT
} port_irq_id_t;

/* --- Per-instance runtime state --- */
typedef struct {
  bool cfg_valid;                      /* Config copy loaded                */
  bool initialized;                    /* Fully configured and running      */
  sim7600_port_config_t cfg;           /* Private copy of user config       */
  volatile uint16 rx_head;             /* ISR writes here                   */
  volatile uint16 rx_tail;             /* App consumes from here            */
  volatile sim7600_port_status_t last; /* Detailed status of last API call  */
  sim7600_port_stats_t stats;          /* Live counters                     */
  port_irq_id_t irq;                   /* CMSIS IRQn enum index             */
} sim7600_port_state_t;

/* ===========================================================================
 *  Factory pin mapping tables - STM32F401RC (LQFP64), DS9716 Rev 11 Table 9
 * ===========================================================================
 *  USART1 (APB2): TX  {PA9 AF7, PB6 AF7}   RX  {PA10 AF7, PB7 AF7}
 *  USART2 (APB1): TX  {PA2 AF7}            RX  {PA3 AF7}
 *    (PD5/PD6 alternatives exist in RM0368 but not on the LQFP64 package)
 *  USART6 (APB2): TX  {PA11 AF8, PC6 AF8}  RX  {PA12 AF8, PC7 AF8}
 *  Flow control (USART1/USART2 only):
 *    USART1: CTS PA11 AF7, RTS PA12 AF7
 *    USART2: CTS PA0  AF7, RTS PA1  AF7   (PD3/PD4 excluded, LQFP100 only)
 * ===========================================================================
 */
static const sim7600_port_pin_map_t m_tx_map[SIM7600_PORT_MAX_INSTANCES][2] = {
    /* USART1 */ {{SIM7600_PIN_PA9, 7U}, {SIM7600_PIN_PB6, 7U}},
    /* USART2 */ {{SIM7600_PIN_PA2, 7U}, {SIM7600_PIN_NONE, 0U}},
    /* USART6 */ {{SIM7600_PIN_PA11, 8U}, {SIM7600_PIN_PC6, 8U}}};

static const sim7600_port_pin_map_t m_rx_map[SIM7600_PORT_MAX_INSTANCES][2] = {
    /* USART1 */ {{SIM7600_PIN_PA10, 7U}, {SIM7600_PIN_PB7, 7U}},
    /* USART2 */ {{SIM7600_PIN_PA3, 7U}, {SIM7600_PIN_NONE, 0U}},
    /* USART6 */ {{SIM7600_PIN_PA12, 8U}, {SIM7600_PIN_PC7, 8U}}};

static const sim7600_port_pin_map_t m_cts_map[SIM7600_PORT_MAX_INSTANCES] = {
    /* USART1 */ {SIM7600_PIN_PA11, 7U},
    /* USART2 */ {SIM7600_PIN_PA0, 7U},
    /* USART6 */ {SIM7600_PIN_NONE, 0U} /* USART6 hardware flow = N.A. */
};

static const sim7600_port_pin_map_t m_rts_map[SIM7600_PORT_MAX_INSTANCES] = {
    /* USART1 */ {SIM7600_PIN_PA12, 7U},
    /* USART2 */ {SIM7600_PIN_PA1, 7U},
    /* USART6 */ {SIM7600_PIN_NONE, 0U} /* USART6 hardware flow = N.A. */
};

/* --- Per-instance storage --- */
static sim7600_port_state_t m_state[SIM7600_PORT_MAX_INSTANCES];

/* --- Cross-instance pin ownership map (prevents silent GPIO conflicts) --- */
static uint16 m_pin_owned[GPIO_PORT_MAX];

/* --- Default instance ring buffer storage (see sim7600_port_cfg.h) --- */
static uint8 m_default_rx_data[SIM7600_PORT_RX_BUFFER_SIZE_DEFAULT];

/* --- Pre-built default configuration (macro-driven from the template) --- */
const sim7600_port_config_t sim7600_port_default_config = {
    .uart_id = SIM7600_PORT_UART_ID_DEFAULT,
    .tx_pin = SIM7600_PORT_TX_PIN_DEFAULT,
    .rx_pin = SIM7600_PORT_RX_PIN_DEFAULT,
    .baud_rate = SIM7600_PORT_BAUD_DEFAULT,
    .word_len = SIM7600_PORT_WORD_LEN_DEFAULT,
    .parity = SIM7600_PORT_PARITY_DEFAULT,
    .stop_bits = SIM7600_PORT_STOP_BITS_DEFAULT,
    .oversampling = SIM7600_PORT_OVERSAMPLE_DEFAULT,
    .flow_control = SIM7600_PORT_FLOW_DEFAULT,
    .cts_pin = SIM7600_PORT_CTS_PIN_DEFAULT,
    .rts_pin = SIM7600_PORT_RTS_PIN_DEFAULT,
    .rx_buffer = &m_default_rx_data[0],
    .rx_buffer_size = SIM7600_PORT_RX_BUFFER_SIZE_DEFAULT,
    .nvic_priority = SIM7600_PORT_NVIC_PRIORITY_DEFAULT};

/*************** Section 5 : Software Interfaces Declarations (APIs)
 * ***************/

/* (public APIs defined below; internal helpers are in Section 6-like body) */

/******************* Section 6 : Private Functions Bodies *******************/

/* ---------------------------------------------------------------------------
 * CMSIS IRQn translation table (STM32F401 vector table):
 *   USART1 = 37 (USART1_IRQn), USART2 = 38 (USART2_IRQn), USART6 = 71 (USART6_IRQn)
 * ------------------------------------------------------------------------- */
static IRQn_Type m_cmsis_irq_of(sim7600_uart_id_t id) {
  static const IRQn_Type lut[3] = {USART1_IRQn, USART2_IRQn, USART6_IRQn};
  return lut[(uint8)id];
}

static port_irq_id_t m_irq_of(sim7600_uart_id_t id) {
  static const port_irq_id_t lut[3] = {PORT_IRQ_USART1, PORT_IRQ_USART2,
                                       PORT_IRQ_USART6};
  return lut[(uint8)id];
}

static USART_TypeDef *m_reg_of(sim7600_uart_id_t id) {
  USART_TypeDef *reg = (id == SIM7600_UART_2)   ? USART2
                       : (id == SIM7600_UART_1) ? USART1
                                                : USART6;
  return reg;
}

/* ---------------------------------------------------------------------------
 * Live bus frequency resolution - reads RCC directly (HSI/HSE/PLL aware).
 * ------------------------------------------------------------------------- */
static uint32 m_sysclk_hz(void) {
  uint32 freq = 0UL;
  uint32 sws = RCC->CFGR & RCC_CFGR_SWS;

  if (RCC_CFGR_SWS_HSI == sws) {
    freq = HSI_VALUE; /* 16 MHz internal     */
  } else if (RCC_CFGR_SWS_HSE == sws) {
    freq = HSE_VALUE;
  } else /* RCC_CFGR_SWS_PLL */
  {
    uint32 pll_src =
        (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) ? HSE_VALUE : HSI_VALUE;
    uint32 pll_m = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM);       /* bits [5:0]   */
    uint32 pll_n = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;  /* bits [14:6]  */
    uint32 pll_p = (RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16; /* bits [17:16]*/
    uint32 pll_pdiv = ((pll_p + 1UL) * 2UL); /* 00->2, 01->4, 10->6, 11->8 */

    if (0UL == pll_m) {
      pll_m = 1UL;
    } /* Defensive guard      */

    freq = (pll_src / pll_m) * pll_n;
    freq /= pll_pdiv;
  }

  if (0UL == freq) {
    freq = HSI_VALUE;
  } /* Undef'd reg fallback */
  return freq;
}

/* --- APB prescaler decode: PPREx 0xx->1, 100->2, 101->4, 110->8, 111->16 ---
 */
static uint32 m_apb_hz(sim7600_uart_id_t id) {
  uint32 sys = m_sysclk_hz();
  uint32 presc_bits = (id == SIM7600_UART_2)
                          ? ((RCC->CFGR & RCC_CFGR_PPRE1) >> 10)
                          : ((RCC->CFGR & RCC_CFGR_PPRE2) >> 13);
  uint32 div = (presc_bits < 4U) ? 1UL : (1UL << (presc_bits - 3UL));
  return sys / div;
}

/* ---------------------------------------------------------------------------
 * Baud rate synthesizer.
 *
 * OVER16: USARTDIV = fPCLK / (16 * baud); BRR[15:4] = mantissa, BRR[3:0] = frac
 * OVER8 : USARTDIV = fPCLK / ( 8 * baud); BRR[3] forced 0, frac on [2:0]
 * ------------------------------------------------------------------------- */
static bool m_brr_calc(uint32 fclk, uint32 baud, bool over8, uint32 *brr) {
  const uint32 scale = over8 ? 8UL : 16UL;
  const uint32 denom = scale * baud;
  uint32 mant = fclk / denom;
  uint32 rem = fclk % denom;
  uint32 frac = ((rem * scale) + (denom / 2UL)) / denom;
  uint32 reg_val;

  if (frac >= scale) {
    mant += 1UL;
    frac -= scale;
  } /* Rounding carry      */

  if (0UL == mant || mant > 0xFFFUL) /* Outside BRR range   */
  {
    return false;
  }

  reg_val = (mant << 4);
  if (over8) {
    frac &= 0x7UL;
  } /* BRR[3] = 0 mandated */
  reg_val |= frac;

  *brr = reg_val;
  return true;
}

/* ---------------------------------------------------------------------------
 * GPIO helpers - raw register level, port/pin resolved at runtime.
 * ------------------------------------------------------------------------- */
static void m_port_clock_enable(uint32 port_idx) {
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN << port_idx);
  (void)RCC->AHB1ENR; /* Clock-enable settling read */
}

static GPIO_TypeDef *m_gpio_of(uint32 port_idx) {
  return (GPIO_TypeDef *)(GPIOA_BASE + (GPIO_PORT_STRIDE * port_idx));
}

static void m_gpio_shadow_pin(uint32 port_idx, uint32 pin_idx) {
  if (port_idx < GPIO_PORT_MAX && pin_idx < GPIO_PIN_MAX) {
    m_pin_owned[port_idx] |= (uint16)(1UL << pin_idx);
  }
}

static void m_gpio_release_pin(uint32 port_idx, uint32 pin_idx) {
  if (port_idx < GPIO_PORT_MAX && pin_idx < GPIO_PIN_MAX) {
    m_pin_owned[port_idx] &= (uint16) ~(1UL << pin_idx);
  }
}

/* --- Ownership check: returns true if any live/held pin conflicts --- */
static bool m_gpio_pin_taken(uint32 port_idx, uint32 pin_idx) {
  uint16 mask = (uint16)(1UL << pin_idx);

  if (port_idx >= GPIO_PORT_MAX || pin_idx >= GPIO_PIN_MAX) {
    return false;
  }

  return (0UL != (m_pin_owned[port_idx] & mask));
}

/*
 * General GPIO configuration.
 *   mode : PORT_GPIO_MODE_INPUT / OUTPUT / AF / ANALOG
 */
static void m_gpio_setup(sim7600_pin_t pin, uint32 mode, uint32 af,
                         uint32 otype, uint32 speed, uint32 pull) {
  GPIO_TypeDef *port = m_gpio_of(GPIO_PIN_PORT(pin));
  uint8 p = GPIO_PIN_INDEX(pin);
  uint8 shift = p * 2U;

  m_port_clock_enable(GPIO_PIN_PORT(pin));

  if (PORT_GPIO_MODE_OUTPUT == mode) {
    port->OTYPER |= (otype << p);
  } else {
    port->OTYPER &= (uint16) ~(1UL << p);
  }

  port->PUPDR = (port->PUPDR & ~(3UL << shift)) | (pull << shift);
  port->OSPEEDR = (port->OSPEEDR & ~(3UL << shift)) | (speed << shift);
  port->MODER = (port->MODER & ~(3UL << shift)) | (mode << shift);

  if (PORT_GPIO_MODE_AF == mode) {
    if (p < 8U) {
      port->AFR[0] =
          (port->AFR[0] & ~(0xFUL << (p * 4U))) | ((uint32)af << (p * 4U));
    } else {
      port->AFR[1] = (port->AFR[1] & ~(0xFUL << ((p - 8U) * 4U))) |
                     ((uint32)af << ((p - 8U) * 4U));
    }
  }
}

/* --- Release a pin back to Analog-In (its power-on default) --- */
static void m_gpio_release(sim7600_pin_t pin) {
  GPIO_TypeDef *port = m_gpio_of(GPIO_PIN_PORT(pin));
  uint8 p = GPIO_PIN_INDEX(pin);

  port->MODER |= (3UL << (p * 2U));
  port->PUPDR &= ~(3UL << (p * 2U));
  m_gpio_release_pin(GPIO_PIN_PORT(pin), p);
}

/* ---------------------------------------------------------------------------
 * Factory pin validation - matches requested TX/RX/flow pin against the
 * DS9716 mapping tables for the requested USART.
 * ------------------------------------------------------------------------- */
static bool m_pin_valid(const sim7600_port_pin_map_t *table, uint8 table_len,
                        sim7600_pin_t pin) {
  for (uint8 i = 0U; i < table_len; i++) {
    if (table[i].pin == pin) {
      return true;
    }
  }
  return false;
}

static sim7600_port_status_t m_conf_validate(const sim7600_port_config_t *cfg) {
  sim7600_port_status_t ret = SIM7600_PORT_STATUS_OK;

  /* --- NULL / resource guards --- */
  if (NULL == cfg || NULL == cfg->rx_buffer || 0U == cfg->rx_buffer_size ||
      0UL == cfg->baud_rate ||
      (uint8)cfg->uart_id >= SIM7600_PORT_MAX_INSTANCES) {
    return SIM7600_PORT_STATUS_PARAM;
  }

  /* --- Buffer sanity: at least 8 bytes, power-of-two not required --- */
  if (cfg->rx_buffer_size < 8U) {
    return SIM7600_PORT_STATUS_PARAM;
  }

  /* --- NVIC preemption priority bounds (group 4) --- */
  if (cfg->nvic_priority > 15U) {
    return SIM7600_PORT_STATUS_PARAM;
  }

  /* --- Pin factory-mapping validation (wisely rejects cross-channel abuse) ---
   */
  if (!m_pin_valid(m_tx_map[(uint8)cfg->uart_id], 2U, cfg->tx_pin)) {
    return SIM7600_PORT_STATUS_INVALID_PIN;
  }
  if (!m_pin_valid(m_rx_map[(uint8)cfg->uart_id], 2U, cfg->rx_pin)) {
    return SIM7600_PORT_STATUS_INVALID_PIN;
  }

  /* --- Acknowledge frame-format extensions properly --- */
  if (SIM7600_WORD_LEN_9 == cfg->word_len &&
      SIM7600_STOP_BITS_1_5 == cfg->stop_bits) {
    return SIM7600_PORT_STATUS_PARAM; /* Illegal combo per RM0368 */
  }

  /* --- Hardware flow control validation --- */
  if (SIM7600_FLOW_NONE != cfg->flow_control) {
    if (SIM7600_UART_6 == cfg->uart_id) {
      return SIM7600_PORT_STATUS_FLOW_INVALID; /* USART6 = N.A. */
    }
    if ((SIM7600_FLOW_CTS == cfg->flow_control ||
         SIM7600_FLOW_CTS_RTS == cfg->flow_control) &&
        !m_pin_valid(&m_cts_map[(uint8)cfg->uart_id], 1U, cfg->cts_pin)) {
      return SIM7600_PORT_STATUS_FLOW_INVALID;
    }
    if ((SIM7600_FLOW_RTS == cfg->flow_control ||
         SIM7600_FLOW_CTS_RTS == cfg->flow_control) &&
        !m_pin_valid(&m_rts_map[(uint8)cfg->uart_id], 1U, cfg->rts_pin)) {
      return SIM7600_PORT_STATUS_FLOW_INVALID;
    }
  }

  /* --- Baud rate feasibility checked against live bus --- */
  {
    uint32 fclk = m_apb_hz(cfg->uart_id);
    uint32 dummy = 0UL;
    if (!m_brr_calc(fclk, cfg->baud_rate,
                    (SIM7600_OVERSAMPLE_8 == cfg->oversampling), &dummy)) {
      return SIM7600_PORT_STATUS_INVALID_BAUD;
    }
  }

  return ret;
}

/* ---------------------------------------------------------------------------
 * Ring buffer push (ISR context only).
 * ------------------------------------------------------------------------- */
static void m_ring_push(sim7600_port_state_t *st, uint8 data) {
  uint32 head = st->rx_head;
  uint32 tail = st->rx_tail;
  uint32 size = st->cfg.rx_buffer_size;
  uint32 used = (head >= tail) ? (head - tail) : (size - tail + head);

  if (used < (uint32)(size - 1U)) /* Keep 1 slot for full/empty */
  {
    st->cfg.rx_buffer[head] = data;
    st->rx_head = (uint16)((head + 1UL >= size) ? 0UL : (head + 1UL));
    st->stats.rx_bytes++;
  } else {
    st->stats.rx_buffer_overflow++;
  }
}

/* ---------------------------------------------------------------------------
 * Ring buffer pop (application context, IRQ-safe).
 * ------------------------------------------------------------------------- */
static bool m_ring_pop(sim7600_port_state_t *st, uint8 *data) {
  bool got = false;
  uint32 prim;
  uint32 head;
  uint32 tail;
  uint32 size;

  PORT_ENTER_CRITICAL(prim);

  head = st->rx_head;
  tail = st->rx_tail;
  size = st->cfg.rx_buffer_size;

  if (head != tail) {
    *data = st->cfg.rx_buffer[tail];
    st->rx_tail = (uint16)((tail + 1UL >= size) ? 0UL : (tail + 1UL));
    got = true;
  }

  PORT_EXIT_CRITICAL(prim);
  return got;
}

/* ---------------------------------------------------------------------------
 * Per-instance ISR body (called from the CMSIS-named handlers).
 * ------------------------------------------------------------------------- */
static void m_irq_dispatch(port_irq_id_t irq) {
  sim7600_port_state_t *st = NULL;
  USART_TypeDef *u = NULL;
  uint32 sr;

  /* Resolve the state by IRQn enum back into instance id */
  for (uint8 i = 0U; i < SIM7600_PORT_MAX_INSTANCES; i++) {
    if (m_state[i].cfg_valid && m_state[i].initialized &&
        (irq == m_state[i].irq)) {
      st = &m_state[i];
      break;
    }
  }

  if (NULL == st) {
    /* Spurious interrupt (e.g. app de-init raced with level-triggered NVIC)
     * - bail out silently to avoid polluting stats of uninitialized state */
    return;
  }

  /* Note: port_irq_id_t mirrors sim7600_uart_id_t numerically (0/1/2)      */
  u = m_reg_of((sim7600_uart_id_t)(uint8)irq);
  sr = u->SR;

  /* --- Hardware RX error flags (all cleared by the SR-then-DR read pair) ---
   */
  if (USART_SR_ORE & sr) {
    st->stats.hw_overrun++;
    st->last = SIM7600_PORT_STATUS_HW_FAULT;
  }
  if (USART_SR_FE & sr) {
    st->stats.hw_framing_errors++;
    st->last = SIM7600_PORT_STATUS_HW_FAULT;
  }
  if (USART_SR_NE & sr) {
    st->stats.hw_noise_errors++;
    st->last = SIM7600_PORT_STATUS_HW_FAULT;
  }
  if (USART_SR_PE & sr) {
    st->stats.hw_parity_errors++;
    st->last = SIM7600_PORT_STATUS_HW_FAULT;
  }

  if (USART_SR_RXNE & sr) {
    m_ring_push(st, (uint8)(u->DR & 0xFFUL));
  } else if (USART_SR_ORE &
             sr) /* Error flag present without RXNE: drain DR anyway */
  {
    (void)u->DR;
  }
}

/******************* Section 7 : CMSIS-named ISRs *******************/

void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART6_IRQHandler(void);

void USART1_IRQHandler(void) { m_irq_dispatch(PORT_IRQ_USART1); }

void USART2_IRQHandler(void) { m_irq_dispatch(PORT_IRQ_USART2); }

void USART6_IRQHandler(void) { m_irq_dispatch(PORT_IRQ_USART6); }

/******************* Section 8 : Public APIs Implementation *******************/

Std_ReturnType sim7600_port_init(const sim7600_port_config_t *cfg) {
  uint32 prim;
  sim7600_port_state_t *st = NULL;
  USART_TypeDef *u = NULL;
  uint32 brr = 0UL;
  uint32 apb = 0UL;
  uint32 cr1 = 0UL;
  uint32 cr2 = 0UL;
  uint32 cr3 = 0UL;
  bool over8 = false;
  Std_ReturnType ret = E_OK;

  /* === 1. Argument validation === */
  {
    sim7600_port_status_t v = m_conf_validate(cfg);

    if (SIM7600_PORT_STATUS_OK != v) {
      if (NULL != cfg && (uint8)cfg->uart_id < SIM7600_PORT_MAX_INSTANCES) {
        m_state[cfg->uart_id].last = v;
      }
      return E_NOT_OK;
    }
  }

  /* === 2. Copy + instance lock === */
  st = &m_state[cfg->uart_id];
  u = m_reg_of(cfg->uart_id);
  over8 = (SIM7600_OVERSAMPLE_8 == cfg->oversampling);
  apb = m_apb_hz(cfg->uart_id);

  PORT_ENTER_CRITICAL(prim);

  if (st->initialized) {
    st->last = SIM7600_PORT_STATUS_BUSY;
    PORT_EXIT_CRITICAL(prim);
    return E_NOT_OK;
  }

  PORT_EXIT_CRITICAL(prim);

  /* === 3. Peripheral clock enable (APB2 : USART1/6 | APB1 : USART2) === */
  if (SIM7600_UART_2 == cfg->uart_id) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR; /* Clock-enable settling read */
  } else {
    RCC->APB2ENR |= (SIM7600_UART_1 == cfg->uart_id) ? RCC_APB2ENR_USART1EN
                                                     : RCC_APB2ENR_USART6EN;
    (void)RCC->APB2ENR;
  }

  /* === 4. Pin ownership arbitration (cross-instance aliasing protection) ===
   */
  if (m_gpio_pin_taken(GPIO_PIN_PORT(cfg->tx_pin),
                       GPIO_PIN_INDEX(cfg->tx_pin)) ||
      m_gpio_pin_taken(GPIO_PIN_PORT(cfg->rx_pin),
                       GPIO_PIN_INDEX(cfg->rx_pin))) {
    st->last = SIM7600_PORT_STATUS_BUSY;
    return E_NOT_OK;
  }
  if (SIM7600_FLOW_CTS == cfg->flow_control ||
      SIM7600_FLOW_CTS_RTS == cfg->flow_control) {
    if (m_gpio_pin_taken(GPIO_PIN_PORT(cfg->cts_pin),
                         GPIO_PIN_INDEX(cfg->cts_pin))) {
      st->last = SIM7600_PORT_STATUS_BUSY;
      return E_NOT_OK;
    }
  }
  if (SIM7600_FLOW_RTS == cfg->flow_control ||
      SIM7600_FLOW_CTS_RTS == cfg->flow_control) {
    if (m_gpio_pin_taken(GPIO_PIN_PORT(cfg->rts_pin),
                         GPIO_PIN_INDEX(cfg->rts_pin))) {
      st->last = SIM7600_PORT_STATUS_BUSY;
      return E_NOT_OK;
    }
  }

  /* === 5. GPIO AF configuration (real AF resolved from factory tables) === */
  {
    uint8 tx_af = 0U;
    uint8 rx_af = 0U;

    for (uint8 i = 0U; i < 2U; i++) {
      if (cfg->tx_pin == m_tx_map[(uint8)cfg->uart_id][i].pin) {
        tx_af = m_tx_map[(uint8)cfg->uart_id][i].af;
      }
      if (cfg->rx_pin == m_rx_map[(uint8)cfg->uart_id][i].pin) {
        rx_af = m_rx_map[(uint8)cfg->uart_id][i].af;
      }
    }

    /* TX pin : Alternate-Function, Push-Pull, High speed, no pull      */
    m_gpio_setup(cfg->tx_pin, PORT_GPIO_MODE_AF, tx_af,
                 PORT_GPIO_OTYPE_PUSH_PULL, PORT_GPIO_SPEED_HIGH,
                 PORT_GPIO_PULL_NONE);
    /* RX pin : Alternate-Function input path (pull-up guards the line
     * against floating input if the SIM7600 side is ever disconnected) */
    m_gpio_setup(cfg->rx_pin, PORT_GPIO_MODE_AF, rx_af,
                 PORT_GPIO_OTYPE_PUSH_PULL, PORT_GPIO_SPEED_HIGH,
                 PORT_GPIO_PULL_UP);
  }

  if (SIM7600_FLOW_CTS == cfg->flow_control ||
      SIM7600_FLOW_CTS_RTS == cfg->flow_control) {
    m_gpio_setup(cfg->cts_pin, PORT_GPIO_MODE_AF,
                 m_cts_map[(uint8)cfg->uart_id].af, PORT_GPIO_OTYPE_PUSH_PULL,
                 PORT_GPIO_SPEED_HIGH, PORT_GPIO_PULL_NONE);
  }
  if (SIM7600_FLOW_RTS == cfg->flow_control ||
      SIM7600_FLOW_CTS_RTS == cfg->flow_control) {
    m_gpio_setup(cfg->rts_pin, PORT_GPIO_MODE_AF,
                 m_rts_map[(uint8)cfg->uart_id].af, PORT_GPIO_OTYPE_PUSH_PULL,
                 PORT_GPIO_SPEED_HIGH, PORT_GPIO_PULL_NONE);
  }

  /* === 6. USART frame format === */
  /* CR2: stop bits */
  switch (cfg->stop_bits) {
  case SIM7600_STOP_BITS_1_5:
    cr2 = PORT_USART_CR2_STOP_1_5 << 12;
    break;
  case SIM7600_STOP_BITS_2:
    cr2 = PORT_USART_CR2_STOP_2 << 12;
    break;
  default:
    cr2 = 0UL;
    break;
  }

  /* CR3: hardware flow control */
  if (SIM7600_FLOW_CTS == cfg->flow_control ||
      SIM7600_FLOW_CTS_RTS == cfg->flow_control) {
    cr3 |= USART_CR3_CTSE;
  }
  if (SIM7600_FLOW_RTS == cfg->flow_control ||
      SIM7600_FLOW_CTS_RTS == cfg->flow_control) {
    cr3 |= USART_CR3_RTSE;
  }

  /* CR1: word len, parity, oversampling, TX/RX enables */
  if (SIM7600_WORD_LEN_9 == cfg->word_len) {
    cr1 |= USART_CR1_M;
  }
  if (SIM7600_PARITY_EVEN == cfg->parity) {
    cr1 |= USART_CR1_PCE;
  }
  if (SIM7600_PARITY_ODD == cfg->parity) {
    cr1 |= USART_CR1_PCE | USART_CR1_PS;
  }
  if (over8) {
    cr1 |= (1UL << 15);
  } /* OVER8 */
  cr1 |= USART_CR1_TE | USART_CR1_RE;

  /* === 7. Baud rate synthesizer + RM0368-required bring-up order === */
  if (!m_brr_calc(apb, cfg->baud_rate, over8, &brr)) {
    st->last = SIM7600_PORT_STATUS_INVALID_BAUD;
    return E_NOT_OK;
  }

  u->CR1 = 0UL; /* UE = 0: configuration unlocked  */
  u->CR2 = cr2;
  u->CR3 = cr3;
  u->CR1 = cr1; /* TE + RE only                    */
  u->BRR = brr;
  u->CR1 = cr1 | USART_CR1_UE | USART_CR1_RXNEIE; /* Enable last    */

  /* === 8. Commit state (MUST precede NVIC enable: if bytes land in that
   *       window, the ISR finds a fully-formed state and serves them
   *       cleanly instead of hitting the spurious-interrupt path) === */
  PORT_ENTER_CRITICAL(prim);

  st->cfg = *cfg; /* Full private copy (pins, buffer ptr, etc.) */
  st->rx_head = 0U;
  st->rx_tail = 0U;
  st->last = SIM7600_PORT_STATUS_OK;
  st->stats = (sim7600_port_stats_t){0};
  st->irq = m_irq_of(cfg->uart_id);
  st->cfg_valid = true;
  st->initialized = true;

  PORT_EXIT_CRITICAL(prim);

  /* === 9. NVIC === */
  HAL_NVIC_SetPriority(m_cmsis_irq_of(cfg->uart_id), cfg->nvic_priority,
                       0);
  HAL_NVIC_EnableIRQ(m_cmsis_irq_of(cfg->uart_id));

  /* === 10. Register owned pins for future arbitration === */
  PORT_ENTER_CRITICAL(prim);

  m_gpio_shadow_pin(GPIO_PIN_PORT(cfg->tx_pin), GPIO_PIN_INDEX(cfg->tx_pin));
  m_gpio_shadow_pin(GPIO_PIN_PORT(cfg->rx_pin), GPIO_PIN_INDEX(cfg->rx_pin));
  if (SIM7600_FLOW_CTS == cfg->flow_control ||
      SIM7600_FLOW_CTS_RTS == cfg->flow_control) {
    m_gpio_shadow_pin(GPIO_PIN_PORT(cfg->cts_pin),
                      GPIO_PIN_INDEX(cfg->cts_pin));
  }
  if (SIM7600_FLOW_RTS == cfg->flow_control ||
      SIM7600_FLOW_CTS_RTS == cfg->flow_control) {
    m_gpio_shadow_pin(GPIO_PIN_PORT(cfg->rts_pin),
                      GPIO_PIN_INDEX(cfg->rts_pin));
  }

  PORT_EXIT_CRITICAL(prim);

  return ret;
}

Std_ReturnType sim7600_port_deinit(sim7600_uart_id_t uart_id) {
  uint32 prim;
  sim7600_port_state_t *st;
  USART_TypeDef *u;
  RCC_TypeDef *rcc;
  uint32 irq_bit;

  if ((uint8)uart_id >= SIM7600_PORT_MAX_INSTANCES) {
    return E_NOT_OK;
  }

  st = &m_state[uart_id];
  u = m_reg_of(uart_id);
  rcc = RCC;
  irq_bit = 0UL;

  PORT_ENTER_CRITICAL(prim);

  if (!st->initialized) {
    st->last = SIM7600_PORT_STATUS_NOT_INIT;
    PORT_EXIT_CRITICAL(prim);
    return E_NOT_OK;
  }

  PORT_EXIT_CRITICAL(prim);

  /* Disable the NVIC line first so no late RX can re-enable it */
  HAL_NVIC_DisableIRQ(m_cmsis_irq_of(uart_id));

  /* Peripheral reset + disable */
  u->CR1 = 0UL;
  u->CR2 = 0UL;
  u->CR3 = 0UL;

  switch (uart_id) {
  case SIM7600_UART_1:
    rcc->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    rcc->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
    irq_bit = RCC_APB2ENR_USART1EN;
    break;
  case SIM7600_UART_2:
    rcc->APB1RSTR |= RCC_APB1RSTR_USART2RST;
    rcc->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
    irq_bit = RCC_APB1ENR_USART2EN;
    break;
  default:
    rcc->APB2RSTR |= RCC_APB2RSTR_USART6RST;
    rcc->APB2RSTR &= ~RCC_APB2RSTR_USART6RST;
    irq_bit = RCC_APB2ENR_USART6EN;
    break;
  }

  /* Peripheral clock off */
  if (SIM7600_UART_2 == uart_id) {
    rcc->APB1ENR &= ~irq_bit;
  } else {
    rcc->APB2ENR &= ~irq_bit;
  }
  (void)rcc->APB1ENR;
  (void)rcc->APB2ENR; /* Settling reads */

  /* GPIO back to Analog-In */
  m_gpio_release(st->cfg.tx_pin);
  m_gpio_release(st->cfg.rx_pin);
  if (SIM7600_FLOW_CTS == st->cfg.flow_control ||
      SIM7600_FLOW_CTS_RTS == st->cfg.flow_control) {
    m_gpio_release(st->cfg.cts_pin);
  }
  if (SIM7600_FLOW_RTS == st->cfg.flow_control ||
      SIM7600_FLOW_CTS_RTS == st->cfg.flow_control) {
    m_gpio_release(st->cfg.rts_pin);
  }

  PORT_ENTER_CRITICAL(prim);

  st->initialized = false;
  st->cfg_valid = false;
  st->rx_head = 0U;
  st->rx_tail = 0U;
  st->last = SIM7600_PORT_STATUS_OK;

  PORT_EXIT_CRITICAL(prim);
  return E_OK;
}

Std_ReturnType sim7600_port_is_init(sim7600_uart_id_t uart_id, bool *is_init) {
  if ((uint8)uart_id >= SIM7600_PORT_MAX_INSTANCES || NULL == is_init) {
    return E_NOT_OK;
  }

  *is_init = m_state[uart_id].initialized;
  return E_OK;
}

Std_ReturnType sim7600_port_send_byte(sim7600_uart_id_t uart_id, uint8 data,
                                      uint32 timeout_ms) {
  return sim7600_port_send(uart_id, &data, 1U, timeout_ms);
}

Std_ReturnType sim7600_port_send(sim7600_uart_id_t uart_id, const uint8 *data,
                                 uint16 len, uint32 timeout_ms) {
  uint32 prim;
  sim7600_port_state_t *st;
  USART_TypeDef *u;
  uint32 start;
  uint32 elapsed;
  uint16 sent;

  if ((uint8)uart_id >= SIM7600_PORT_MAX_INSTANCES || NULL == data ||
      0U == len) {
    return E_NOT_OK;
  }

  st = &m_state[uart_id];
  u = m_reg_of(uart_id);

  PORT_ENTER_CRITICAL(prim);
  if (!st->initialized) {
    st->last = SIM7600_PORT_STATUS_NOT_INIT;
    PORT_EXIT_CRITICAL(prim);
    return E_NOT_OK;
  }
  PORT_EXIT_CRITICAL(prim);

  start = HAL_GetTick();
  elapsed = 0UL;
  sent = 0U;

  /* --- Payload bytes (TXE = "Data Register can accept a new byte") --- */
  while (sent < len) {
    while (0UL == (u->SR & USART_SR_TXE)) {
      elapsed = HAL_GetTick() - start;
      /* timeout_ms = 0 (NO_TIMEOUT) => one flag poll, then bail out */
      if (elapsed >= timeout_ms) {
        st->last = SIM7600_PORT_STATUS_TIMEOUT;
        return E_NOT_OK;
      }
    }

    u->DR = (uint32)data[sent];
    sent++;
    st->stats.tx_bytes++;
  }

  /* --- Wait for the shift register to drain (TC) so callers can safely
   *     follow up with actions like a baud change (AT+IPR) or low-power
   *     entry right after - nothing gets cut mid-flight. --- */
  while (0UL == (u->SR & USART_SR_TC)) {
    elapsed = HAL_GetTick() - start;
    if (elapsed >= timeout_ms) {
      st->last = SIM7600_PORT_STATUS_TIMEOUT;
      return E_NOT_OK;
    }
  }

  st->last = SIM7600_PORT_STATUS_OK;
  return E_OK;
}

Std_ReturnType sim7600_port_send_ascii(sim7600_uart_id_t uart_id,
                                       const char *str, uint32 timeout_ms) {
  if (NULL == str) {
    return E_NOT_OK;
  }
  return sim7600_port_send(uart_id, (const uint8 *)str, (uint16)strlen(str),
                           timeout_ms);
}

Std_ReturnType sim7600_port_recv_byte(sim7600_uart_id_t uart_id, uint8 *data,
                                      uint32 timeout_ms) {
  uint16 gotg = 0U;
  Std_ReturnType ret;

  if (NULL == data) {
    return E_NOT_OK;
  }

  ret = sim7600_port_recv(uart_id, data, 1U, &gotg, timeout_ms);

  if (E_OK == ret && 1U != gotg) {
    /* Defensive: recv() makes this impossible; guard against regression */
    return E_NOT_OK;
  }
  return ret;
}

Std_ReturnType sim7600_port_recv(sim7600_uart_id_t uart_id, uint8 *buf,
                                 uint16 max_len, uint16 *received,
                                 uint32 timeout_ms) {
  uint32 prim;
  sim7600_port_state_t *st;
  uint32 start;

  if ((uint8)uart_id >= SIM7600_PORT_MAX_INSTANCES || NULL == buf ||
      0U == max_len || NULL == received) {
    return E_NOT_OK;
  }

  st = &m_state[uart_id];

  PORT_ENTER_CRITICAL(prim);
  if (!st->initialized) {
    st->last = SIM7600_PORT_STATUS_NOT_INIT;
    PORT_EXIT_CRITICAL(prim);
    return E_NOT_OK;
  }
  PORT_EXIT_CRITICAL(prim);

  start = HAL_GetTick();
  *received = 0U;

  while (*received < max_len) {
    if (m_ring_pop(st, &buf[*received])) {
      (*received)++;
    } else {
      uint32 elapsed = HAL_GetTick() - start;
      /* timeout_ms = 0 (NO_TIMEOUT) => one buffer poll, then return     */
      if (elapsed >= timeout_ms) {
        break; /* Deadline hit: return what's in hand               */
      }
    }
  }

  if (*received > 0U) {
    st->last = SIM7600_PORT_STATUS_OK;
    return E_OK; /* Got some data: success                     */
  }

  /* Nothing arrived inside the window */
  st->last = SIM7600_PORT_STATUS_TIMEOUT;
  return E_NOT_OK;
}

Std_ReturnType sim7600_port_rx_available(sim7600_uart_id_t uart_id,
                                         uint16 *count) {
  uint32 prim;
  sim7600_port_state_t *st;
  uint32 head;
  uint32 tail;
  uint32 size;

  if ((uint8)uart_id >= SIM7600_PORT_MAX_INSTANCES || NULL == count) {
    return E_NOT_OK;
  }

  st = &m_state[uart_id];

  PORT_ENTER_CRITICAL(prim);

  if (!st->initialized) {
    st->last = SIM7600_PORT_STATUS_NOT_INIT;
    PORT_EXIT_CRITICAL(prim);
    return E_NOT_OK;
  }

  head = st->rx_head;
  tail = st->rx_tail;
  size = st->cfg.rx_buffer_size;

  PORT_EXIT_CRITICAL(prim);

  *count = (uint16)((head >= tail) ? (head - tail) : (size - tail + head));
  return E_OK;
}

Std_ReturnType sim7600_port_rx_peek(sim7600_uart_id_t uart_id, uint8 *data,
                                    uint16 offset) {
  uint32 prim;
  sim7600_port_state_t *st;
  uint32 head;
  uint32 tail;
  uint32 size;
  uint32 used;
  uint32 pos;

  if ((uint8)uart_id >= SIM7600_PORT_MAX_INSTANCES || NULL == data) {
    return E_NOT_OK;
  }

  st = &m_state[uart_id];

  PORT_ENTER_CRITICAL(prim);

  if (!st->initialized) {
    st->last = SIM7600_PORT_STATUS_NOT_INIT;
    PORT_EXIT_CRITICAL(prim);
    return E_NOT_OK;
  }

  head = st->rx_head;
  tail = st->rx_tail;
  size = st->cfg.rx_buffer_size;
  used = (head >= tail) ? (head - tail) : (size - tail + head);

  if ((uint32)offset >= used) {
    st->last = SIM7600_PORT_STATUS_PARAM;
    PORT_EXIT_CRITICAL(prim);
    return E_NOT_OK;
  }

  pos = tail + (uint32)offset;
  if (pos >= size) {
    pos -= size;
  }
  *data = st->cfg.rx_buffer[pos];

  PORT_EXIT_CRITICAL(prim);
  return E_OK;
}

Std_ReturnType sim7600_port_rx_flush(sim7600_uart_id_t uart_id) {
  uint32 prim;
  sim7600_port_state_t *st;

  if ((uint8)uart_id >= SIM7600_PORT_MAX_INSTANCES) {
    return E_NOT_OK;
  }

  st = &m_state[uart_id];

  PORT_ENTER_CRITICAL(prim);

  if (!st->initialized) {
    st->last = SIM7600_PORT_STATUS_NOT_INIT;
    PORT_EXIT_CRITICAL(prim);
    return E_NOT_OK;
  }

  /* Jump the tail straight onto the live head: IRQ still pushes safely   */
  st->rx_tail = st->rx_head;
  st->last = SIM7600_PORT_STATUS_OK;

  PORT_EXIT_CRITICAL(prim);
  return E_OK;
}

Std_ReturnType sim7600_port_get_stats(sim7600_uart_id_t uart_id,
                                      sim7600_port_stats_t *stats) {
  uint32 prim;
  sim7600_port_state_t *st;

  if ((uint8)uart_id >= SIM7600_PORT_MAX_INSTANCES || NULL == stats) {
    return E_NOT_OK;
  }

  st = &m_state[uart_id];

  PORT_ENTER_CRITICAL(prim);

  if (!st->initialized) {
    st->last = SIM7600_PORT_STATUS_NOT_INIT;
    PORT_EXIT_CRITICAL(prim);
    return E_NOT_OK;
  }

  *stats = st->stats;

  PORT_EXIT_CRITICAL(prim);
  return E_OK;
}

Std_ReturnType sim7600_port_reset_stats(sim7600_uart_id_t uart_id) {
  uint32 prim;
  sim7600_port_state_t *st;

  if ((uint8)uart_id >= SIM7600_PORT_MAX_INSTANCES) {
    return E_NOT_OK;
  }

  st = &m_state[uart_id];

  PORT_ENTER_CRITICAL(prim);

  if (!st->initialized) {
    st->last = SIM7600_PORT_STATUS_NOT_INIT;
    PORT_EXIT_CRITICAL(prim);
    return E_NOT_OK;
  }

  st->stats = (sim7600_port_stats_t){0};

  PORT_EXIT_CRITICAL(prim);
  return E_OK;
}

Std_ReturnType sim7600_port_get_last_status(sim7600_uart_id_t uart_id,
                                            sim7600_port_status_t *status) {
  uint32 prim;
  sim7600_port_state_t *st;

  if ((uint8)uart_id >= SIM7600_PORT_MAX_INSTANCES || NULL == status) {
    return E_NOT_OK;
  }

  st = &m_state[uart_id];

  PORT_ENTER_CRITICAL(prim);
  *status = st->last;
  PORT_EXIT_CRITICAL(prim);
  return E_OK;
}
