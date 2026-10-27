/******************* Section 0 : File Header *******************/

/**
 *  hx711.c  -  HX711 24-bit Load Cell ADC Driver (STM32F401RCTx)
 *
 *  Bit-banged 2-wire interface (PD_SCK / DOUT) on any two GPIO pins.
 *    - Microsecond delay from the Cortex-M4 DWT cycle counter.
 *    - Atomic pin access through BSRR / IDR.
 *    - Fixed point only (no floating point): counts-per-kg scale, weight in mg.
 */

/******************* Section 1 : Includes *******************/

#include "../../../Inc/ECUAL/Hx711/hx711.h"

/******************* Section 2 : Macros Declarations *******************/

#define HX711_DATA_BITS             (24U)
#define HX711_PULSE_US              (1U)

/* EMA filter: new = (old * (DEN-NUM) + sample * NUM) / DEN  -> 30 % new sample */
#define HX711_EMA_NUM               (3)
#define HX711_EMA_DEN               (10)

/******************* Section 3 : Variables Definitions *******************/

/* (none) */

/************** Section 4 : Helper Functions Declarations **************/

static void  hx711_dwt_init(void);
static void  hx711_delay_us(uint32 us);
static GPIO_TypeDef *hx711_port_of(uint8 pin_id);
static void  hx711_gpio_clk_enable(const GPIO_TypeDef *port);
static void  hx711_sck_high(const hx711_t *h);
static void  hx711_sck_low(const hx711_t *h);
static uint8 hx711_dout_read(const hx711_t *h);

/*************** Section 5 : Software Interfaces Definitions (APIs) ****************/

Std_ReturnType hx711_init(hx711_t *h, const hx711_cfg_t *cfg)
{
    GPIO_InitTypeDef gpio = {0};

    if ((NULL == h) || (NULL == cfg)) {
        return E_NOT_OK;
    }

    h->sck_port  = hx711_port_of(cfg->sck_pin);
    h->sck_pin   = (uint16)(1U << (cfg->sck_pin & 0x0FU));
    h->dout_port = hx711_port_of(cfg->dout_pin);
    h->dout_pin  = (uint16)(1U << (cfg->dout_pin & 0x0FU));

    if ((NULL == h->sck_port) || (NULL == h->dout_port) ||
        ((h->sck_port == h->dout_port) && (h->sck_pin == h->dout_pin))) {
        return E_NOT_OK;
    }

    h->gain                 = cfg->gain;
    h->offset               = 0;
    h->scale_cpk            = 1;
    h->latest_raw           = 0;
    h->latest_weight_mg     = 0;
    h->latest_weight_valid  = false;

    /* PD_SCK: push-pull output, idle low (chip powered up) */
    hx711_gpio_clk_enable(h->sck_port);
    gpio.Pin   = h->sck_pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(h->sck_port, &gpio);
    hx711_sck_low(h);

    /* DOUT: input with pull-up */
    hx711_gpio_clk_enable(h->dout_port);
    gpio.Pin   = h->dout_pin;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(h->dout_port, &gpio);

    hx711_dwt_init();

    return E_OK;
}

void hx711_task(hx711_t *h)
{
    int32 raw;
    int32 weight_mg;

    if ((NULL == h) || !hx711_is_ready(h)) {
        return;
    }

    /* Readiness was checked above, so this read does not wait. */
    if (E_OK != hx711_read_raw(h, &raw, HX711_READ_TIMEOUT_MS)) {
        return;
    }

    h->latest_raw = raw;

    weight_mg = (int32)(((int64)(raw - h->offset) * 1000000) / (int64)h->scale_cpk);

    if (!h->latest_weight_valid) {
        h->latest_weight_mg    = weight_mg;
        h->latest_weight_valid = true;
    } else {
        /* EMA filter to smooth chassis / motor vibration */
        h->latest_weight_mg = (int32)(((int64)h->latest_weight_mg * (HX711_EMA_DEN - HX711_EMA_NUM) +
                                       (int64)weight_mg * HX711_EMA_NUM) / HX711_EMA_DEN);
    }
}

bool hx711_get_latest_weight_mg(const hx711_t *h, int32 *weight_mg)
{
    if ((NULL == h) || (NULL == weight_mg) || !h->latest_weight_valid) {
        return false;
    }

    *weight_mg = h->latest_weight_mg;
    return true;
}

bool hx711_get_latest_raw(const hx711_t *h, int32 *raw)
{
    if ((NULL == h) || (NULL == raw) || !h->latest_weight_valid) {
        return false;
    }

    *raw = h->latest_raw;
    return true;
}

void hx711_reset_filter(hx711_t *h)
{
    if (NULL != h) {
        h->latest_weight_valid = false;
    }
}

bool hx711_is_ready(const hx711_t *h)
{
    if (NULL == h) {
        return false;
    }
    return (0U == hx711_dout_read(h));
}

Std_ReturnType hx711_read_raw(hx711_t *h, int32 *value, uint32 timeout_ms)
{
    uint32 data = 0U;
    uint32 t0;
    uint32 primask;
    uint8  i;

    if ((NULL == h) || (NULL == value)) {
        return E_NOT_OK;
    }

    /* Wait for DOUT to go low (data ready) */
    t0 = HAL_GetTick();
    while (!hx711_is_ready(h)) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            return E_NOT_OK;
        }
    }

    /* Mask interrupts: PD_SCK must not stay high > 60 us or chip powers down */
    primask = __get_PRIMASK();
    __disable_irq();

    /* 24 data bits, MSB first */
    for (i = 0U; i < HX711_DATA_BITS; i++) {
        hx711_sck_high(h);
        hx711_delay_us(HX711_PULSE_US);
        data = (data << 1) | hx711_dout_read(h);
        hx711_sck_low(h);
        hx711_delay_us(HX711_PULSE_US);
    }

    /* 1-3 extra pulses select gain/channel for the next conversion */
    for (i = 0U; i < (uint8)h->gain; i++) {
        hx711_sck_high(h);
        hx711_delay_us(HX711_PULSE_US);
        hx711_sck_low(h);
        hx711_delay_us(HX711_PULSE_US);
    }

    __set_PRIMASK(primask);

    /* Sign-extend 24-bit two's complement to 32-bit */
    *value = ((int32)(data << 8)) >> 8;

    return E_OK;
}

Std_ReturnType hx711_read_average(hx711_t *h, uint8 samples, int32 *value)
{
    int64 sum = 0;
    int32 v;
    uint8 i;

    if ((NULL == h) || (NULL == value) || (0U == samples)) {
        return E_NOT_OK;
    }

    for (i = 0U; i < samples; i++) {
        if (E_OK != hx711_read_raw(h, &v, HX711_READ_TIMEOUT_MS)) {
            return E_NOT_OK;
        }
        sum += v;
    }

    *value = (int32)(sum / (int64)samples);
    return E_OK;
}

Std_ReturnType hx711_tare(hx711_t *h, uint8 samples)
{
    int32 avg;

    if ((NULL == h) || (E_OK != hx711_read_average(h, samples, &avg))) {
        return E_NOT_OK;
    }

    h->offset = avg;
    hx711_reset_filter(h);          /* drop stale filtered value */
    return E_OK;
}

Std_ReturnType hx711_set_scale(hx711_t *h, int32 counts_per_kg)
{
    if ((NULL == h) || (0 == counts_per_kg)) {
        return E_NOT_OK;
    }

    h->scale_cpk = counts_per_kg;
    hx711_reset_filter(h);
    return E_OK;
}

Std_ReturnType hx711_set_offset(hx711_t *h, int32 offset)
{
    if (NULL == h) {
        return E_NOT_OK;
    }

    h->offset = offset;
    hx711_reset_filter(h);
    return E_OK;
}

Std_ReturnType hx711_calibrate(hx711_t *h, uint32 known_weight_g, uint8 samples)
{
    int32 avg;
    int64 scale;

    if ((NULL == h) || (0U == known_weight_g)) {
        return E_NOT_OK;
    }

    if (E_OK != hx711_read_average(h, samples, &avg)) {
        return E_NOT_OK;
    }

    /* counts per kg = (counts * 1000 g/kg) / known weight in g */
    scale = ((int64)(avg - h->offset) * 1000) / (int64)known_weight_g;

    /* Reject a zero scale (no load detected) or one that overflows int32 */
    if ((0 == scale) || (scale > (int64)INT32_MAX) || (scale < (int64)INT32_MIN)) {
        return E_NOT_OK;
    }

    h->scale_cpk = (int32)scale;
    hx711_reset_filter(h);
    return E_OK;
}

Std_ReturnType hx711_get_counts(hx711_t *h, uint8 samples, int32 *counts)
{
    int32 avg;

    if ((NULL == h) || (NULL == counts)) {
        return E_NOT_OK;
    }

    if (E_OK != hx711_read_average(h, samples, &avg)) {
        return E_NOT_OK;
    }

    *counts = avg - h->offset;
    return E_OK;
}

Std_ReturnType hx711_get_weight_mg(hx711_t *h, uint8 samples, int32 *weight_mg)
{
    int32 counts;

    if ((NULL == h) || (NULL == weight_mg)) {
        return E_NOT_OK;
    }

    if (E_OK != hx711_get_counts(h, samples, &counts)) {
        return E_NOT_OK;
    }

    *weight_mg = (int32)(((int64)counts * 1000000) / (int64)h->scale_cpk);
    return E_OK;
}

Std_ReturnType hx711_set_gain(hx711_t *h, hx711_gain_t gain)
{
    int32 dummy;

    if (NULL == h) {
        return E_NOT_OK;
    }

    h->gain = gain;
    hx711_reset_filter(h);

    /* One read clocks out the new gain pulse count; next sample uses it */
    return hx711_read_raw(h, &dummy, HX711_READ_TIMEOUT_MS);
}

void hx711_power_down(hx711_t *h)
{
    if (NULL == h) {
        return;
    }

    hx711_sck_low(h);
    hx711_delay_us(1U);
    hx711_sck_high(h);
    hx711_delay_us(100U);    /* > 60 us => power down; leave SCK high */
}

Std_ReturnType hx711_power_up(hx711_t *h)
{
    int32 dummy;

    if (NULL == h) {
        return E_NOT_OK;
    }

    hx711_sck_low(h);
    hx711_reset_filter(h);

    /* Chip resets to A/128 on wake-up; re-apply the stored gain with one dummy read. */
    return hx711_read_raw(h, &dummy, HX711_READ_TIMEOUT_MS);
}

/************** Section 6 : Helper Functions Definitions **************/

static void hx711_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void hx711_delay_us(uint32 us)
{
    uint32 start  = DWT->CYCCNT;
    uint32 cycles = us * (SystemCoreClock / 1000000U);
    volatile uint32 timeout = cycles * 10U + 50U;

    while (((DWT->CYCCNT - start) < cycles) && (--timeout > 0U)) {
        /* wait */
    }
}

static GPIO_TypeDef *hx711_port_of(uint8 pin_id)
{
    switch (pin_id >> 4U) {
        case HX711_PORT_A: return GPIOA;
        case HX711_PORT_B: return GPIOB;
        case HX711_PORT_C: return GPIOC;
        case HX711_PORT_D: return GPIOD;
        case HX711_PORT_E: return GPIOE;
        case HX711_PORT_H: return GPIOH;
        default:           return NULL;
    }
}

static void hx711_gpio_clk_enable(const GPIO_TypeDef *port)
{
    if      (port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
    else if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
    else if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
    else if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
    else if (port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
    else if (port == GPIOH) { __HAL_RCC_GPIOH_CLK_ENABLE(); }
    else                    { /* unknown port: leave clocks untouched */ }
}

static void hx711_sck_high(const hx711_t *h)
{
    h->sck_port->BSRR = (uint32)h->sck_pin;
}

static void hx711_sck_low(const hx711_t *h)
{
    h->sck_port->BSRR = ((uint32)h->sck_pin) << 16U;
}

static uint8 hx711_dout_read(const hx711_t *h)
{
    return (0U != (h->dout_port->IDR & h->dout_pin)) ? 1U : 0U;
}
