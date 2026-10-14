/******************* Section 0 : Includes *******************/

#include "ecual_init.h"
#include "esp_log.h"

/******************* Section 1 : Variables Definitions *******************/

static const char *TAG = "ECUAL_INIT";


/* ---- HX711 + 50Kg Load Cell : Shelf Weight Sensor ---- */
hx711_config_t loadcell = {
    .dout_pin = GPIO_NUM_16,
    .sck_pin = GPIO_NUM_17,
    .gain = HX711_GAIN_128,
    .offset = 0,
    .scale = 22800.0f,               /* Counts per kg (placeholder, calibrate with known weight) */
    .max_capacity_kg = 50.0f,        /* 50Kg rated maximum capacity */
    .overload_margin_kg = 2.0f       /* 52Kg safe limit */
};


/************** Section 2 : Helper Functions Declarations **************/

/*************** Section 3 : Software Interfaces Definitions (APIs) ****************/

Std_ReturnType ecual_init(void) {
    /* Each driver is checked independently and logged by name instead of
     * being folded into one bitwise flag: the previous `Retval |= x`
     * pattern could never report failure once Retval started at E_OK(1),
     * since 1 | 0 == 1. A missing/unwired sensor should not brick the
     * whole board — we log it and keep going so the rest of the system
     * (and MQTT connectivity) still comes up. */
    Std_ReturnType overall = E_OK;
    if (E_OK != ecual_hx711_init(&loadcell)) {
        ESP_LOGW(TAG, "HX711 init failed (DOUT=GPIO%d SCK=GPIO%d)", loadcell.dout_pin, loadcell.sck_pin);
        overall = E_NOT_OK;
    }

    if (E_OK != overall) {
        ESP_LOGW(TAG, "One or more ECUAL drivers failed to init — continuing in degraded mode");
    }
    return overall;
}

/************** Section 4 : Helper Functions Definitions **************/
