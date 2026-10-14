/******************* Section 0 : Includes *******************/

#include "ecual_hx711.h"

/******************* Section 1 : Variables Definitions *******************/

static portMUX_TYPE hx711_spinlock = portMUX_INITIALIZER_UNLOCKED;

/************** Section 2 : Helper Functions Declarations **************/

static Std_ReturnType hx711_wait_ready(const hx711_config_t *hx711, uint32 timeout_ms);
static void hx711_sort_samples(int32 *arr, uint8 n);

/*************** Section 3 : Software Interfaces Definitions (APIs) ***************/

Std_ReturnType ecual_hx711_init(hx711_config_t *hx711) {
    Std_ReturnType Retval = E_OK;
    if (NULL == hx711) {
        Retval = E_NOT_OK;
    } else {
        /* Configure DOUT pin as Input with Pull-Up */
        gpio_config_t dout_cfg = {
            .pin_bit_mask = (1ULL << hx711->dout_pin),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE
        };
        esp_err_t err1 = gpio_config(&dout_cfg);

        /* Configure SCK pin as Output */
        gpio_config_t sck_cfg = {
            .pin_bit_mask = (1ULL << hx711->sck_pin),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE
        };
        esp_err_t err2 = gpio_config(&sck_cfg);

        if ((ESP_OK != err1) || (ESP_OK != err2)) {
            Retval = E_NOT_OK;
        } else {
            gpio_set_level(hx711->sck_pin, 0);

            if (hx711->gain < HX711_GAIN_128 || hx711->gain > HX711_GAIN_64) {
                hx711->gain = HX711_GAIN_128;
            }
            if (hx711->scale == 0.0f) {
                hx711->scale = 1.0f;
            }
            if (hx711->max_capacity_kg <= 0.0f) {
                hx711->max_capacity_kg = HX711_RATED_CAPACITY_KG_DEFAULT;
            }
            if (hx711->overload_margin_kg <= 0.0f) {
                hx711->overload_margin_kg = HX711_OVERLOAD_MARGIN_KG_DEFAULT;
            }

            /* Perform dummy read to initialize gain mode in HX711 hardware */
            int32 dummy = 0;
            ecual_hx711_read_raw(hx711, &dummy);

            /* Auto-tare zero offset baseline if unconfigured */
            if (hx711->offset == 0) {
                ecual_hx711_tare(hx711, 10);
            }
        }
    }
    return Retval;
}

Std_ReturnType ecual_hx711_is_ready(hx711_config_t *hx711, bool *is_ready) {
    Std_ReturnType Retval = E_OK;
    if ((NULL == hx711) || (NULL == is_ready)) {
        Retval = E_NOT_OK;
    } else {
        *is_ready = (gpio_get_level(hx711->dout_pin) == 0);
    }
    return Retval;
}

Std_ReturnType ecual_hx711_read_raw(hx711_config_t *hx711, int32 *raw_value) {
    Std_ReturnType Retval = E_OK;
    if ((NULL == hx711) || (NULL == raw_value)) {
        Retval = E_NOT_OK;
    } else {
        Retval = hx711_wait_ready(hx711, 200);
        if (E_OK == Retval) {
            uint32 data = 0;

            /* Critical section prevents task preemption while SCK is HIGH,
             * avoiding unwanted HX711 power-down (>60us SCK high duration). */
            portENTER_CRITICAL(&hx711_spinlock);

            /* Pulse SCK 24 times to shift out 24 bits of data */
            for (int i = 0; i < 24; i++) {
                gpio_set_level(hx711->sck_pin, 1);
                ets_delay_us(1);
                data = (data << 1) | (uint32)gpio_get_level(hx711->dout_pin);
                gpio_set_level(hx711->sck_pin, 0);
                ets_delay_us(1);
            }

            /* Send additional pulses to set gain for next conversion */
            uint8 extra_pulses = (uint8)hx711->gain;
            for (uint8 i = 0; i < extra_pulses; i++) {
                gpio_set_level(hx711->sck_pin, 1);
                ets_delay_us(1);
                gpio_set_level(hx711->sck_pin, 0);
                ets_delay_us(1);
            }

            portEXIT_CRITICAL(&hx711_spinlock);

            /* Sign-extend 24-bit 2's complement value to 32-bit signed integer */
            if (data & 0x800000U) {
                data |= 0xFF000000U;
            }
            *raw_value = (int32)data;
        }
    }
    return Retval;
}

Std_ReturnType ecual_hx711_read_average(hx711_config_t *hx711, uint8 times, int32 *avg_value) {
    Std_ReturnType Retval = E_OK;
    if ((NULL == hx711) || (NULL == avg_value) || (times == 0)) {
        Retval = E_NOT_OK;
    } else {
        #define MAX_SAMPLES_BUFFER 32
        uint8 sample_count = (times > MAX_SAMPLES_BUFFER) ? MAX_SAMPLES_BUFFER : times;
        int32 samples[MAX_SAMPLES_BUFFER];
        uint8 valid_reads = 0;

        for (uint8 i = 0; i < sample_count; i++) {
            int32 raw = 0;
            if (E_OK == ecual_hx711_read_raw(hx711, &raw)) {
                samples[valid_reads++] = raw;
            }
        }

        if (valid_reads == 0) {
            Retval = E_NOT_OK;
        } else if (valid_reads >= 5) {
            /* Trimmed mean: discard highest and lowest outliers to reject vibration/noise */
            hx711_sort_samples(samples, valid_reads);
            int64 sum = 0;
            uint8 trim = 1;
            for (uint8 i = trim; i < (valid_reads - trim); i++) {
                sum += samples[i];
            }
            *avg_value = (int32)(sum / (valid_reads - (2 * trim)));
        } else {
            /* Simple average for few samples */
            int64 sum = 0;
            for (uint8 i = 0; i < valid_reads; i++) {
                sum += samples[i];
            }
            *avg_value = (int32)(sum / valid_reads);
        }
    }
    return Retval;
}

Std_ReturnType ecual_hx711_read_data(hx711_config_t *hx711, int32 *raw_value, float *weight, uint8 times) {
    Std_ReturnType Retval = E_OK;
    int32 avg_raw = 0;
    if ((NULL == hx711) || ((NULL == raw_value) && (NULL == weight))) {
        Retval = E_NOT_OK;
    } else {
        Retval = ecual_hx711_read_average(hx711, times, &avg_raw);
        if (E_OK == Retval) {
            if (NULL != raw_value) {
                *raw_value = avg_raw;
            }
            if (NULL != weight) {
                if (hx711->scale == 0.0f) {
                    hx711->scale = 1.0f;
                }
                *weight = ((float)avg_raw - (float)hx711->offset) / hx711->scale;
            }
        }
    }
    return Retval;
}

Std_ReturnType ecual_hx711_tare(hx711_config_t *hx711, uint8 times) {
    Std_ReturnType Retval = E_OK;
    int32 avg = 0;
    if (NULL == hx711) {
        Retval = E_NOT_OK;
    } else {
        Retval = ecual_hx711_read_average(hx711, times, &avg);
        if (E_OK == Retval) {
            hx711->offset = avg;
        }
    }
    return Retval;
}

Std_ReturnType ecual_hx711_get_units(hx711_config_t *hx711, uint8 times, float *weight) {
    Std_ReturnType Retval = E_OK;
    int32 avg_raw = 0;
    if ((NULL == hx711) || (NULL == weight)) {
        Retval = E_NOT_OK;
    } else {
        Retval = ecual_hx711_read_average(hx711, times, &avg_raw);
        if (E_OK == Retval) {
            if (hx711->scale == 0.0f) {
                hx711->scale = 1.0f;
            }
            *weight = ((float)avg_raw - (float)hx711->offset) / hx711->scale;
        }
    }
    return Retval;
}

Std_ReturnType ecual_hx711_power_down(hx711_config_t *hx711) {
    Std_ReturnType Retval = E_OK;
    if (NULL == hx711) {
        Retval = E_NOT_OK;
    } else {
        gpio_set_level(hx711->sck_pin, 0);
        gpio_set_level(hx711->sck_pin, 1);
        ets_delay_us(80); /* > 60us puts HX711 into power down mode */
    }
    return Retval;
}

Std_ReturnType ecual_hx711_power_up(hx711_config_t *hx711) {
    Std_ReturnType Retval = E_OK;
    if (NULL == hx711) {
        Retval = E_NOT_OK;
    } else {
        gpio_set_level(hx711->sck_pin, 0);
    }
    return Retval;
}

/* ================= 50Kg Load Cell Specific APIs ================= */

Std_ReturnType ecual_hx711_get_weight_kg(hx711_config_t *hx711, uint8 times, float *weight_kg) {
    Std_ReturnType Retval = E_OK;
    if ((NULL == hx711) || (NULL == weight_kg)) {
        Retval = E_NOT_OK;
    } else {
        float weight = 0.0f;
        Retval = ecual_hx711_get_units(hx711, times, &weight);
        if (E_OK == Retval) {
            *weight_kg = weight;
            /* Overload or severe negative reading verification */
            float max_limit = hx711->max_capacity_kg + hx711->overload_margin_kg;
            if ((*weight_kg > max_limit) || (*weight_kg < HX711_UNDERLOAD_LIMIT_KG)) {
                Retval = E_NOT_OK; /* Out of safe calibrated bounds */
            }
        }
    }
    return Retval;
}

Std_ReturnType ecual_hx711_get_weight_g(hx711_config_t *hx711, uint8 times, float *weight_g) {
    Std_ReturnType Retval = E_OK;
    if ((NULL == hx711) || (NULL == weight_g)) {
        Retval = E_NOT_OK;
    } else {
        float kg = 0.0f;
        Retval = ecual_hx711_get_weight_kg(hx711, times, &kg);
        *weight_g = kg * 1000.0f;
    }
    return Retval;
}

Std_ReturnType ecual_hx711_is_overloaded(hx711_config_t *hx711, uint8 times, bool *is_overloaded) {
    Std_ReturnType Retval = E_OK;
    if ((NULL == hx711) || (NULL == is_overloaded)) {
        Retval = E_NOT_OK;
    } else {
        float weight = 0.0f;
        Retval = ecual_hx711_get_units(hx711, times, &weight);
        if (E_OK == Retval) {
            *is_overloaded = (weight > (hx711->max_capacity_kg + hx711->overload_margin_kg));
        }
    }
    return Retval;
}

Std_ReturnType ecual_hx711_calibrate_scale(hx711_config_t *hx711, float known_weight_kg, uint8 times, float *calculated_scale) {
    Std_ReturnType Retval = E_OK;
    if ((NULL == hx711) || (known_weight_kg <= 0.0f)) {
        Retval = E_NOT_OK;
    } else {
        int32 avg_raw = 0;
        Retval = ecual_hx711_read_average(hx711, times, &avg_raw);
        if (E_OK == Retval) {
            float new_scale = ((float)avg_raw - (float)hx711->offset) / known_weight_kg;
            if (new_scale != 0.0f) {
                hx711->scale = new_scale;
                if (NULL != calculated_scale) {
                    *calculated_scale = new_scale;
                }
            } else {
                Retval = E_NOT_OK;
            }
        }
    }
    return Retval;
}

Std_ReturnType ecual_hx711_set_scale(hx711_config_t *hx711, float scale) {
    if ((NULL == hx711) || (scale == 0.0f)) {
        return E_NOT_OK;
    }
    hx711->scale = scale;
    return E_OK;
}

Std_ReturnType ecual_hx711_set_offset(hx711_config_t *hx711, int32 offset) {
    if (NULL == hx711) {
        return E_NOT_OK;
    }
    hx711->offset = offset;
    return E_OK;
}

/************** Section 4 : Helper Functions Definitions **************/

static Std_ReturnType hx711_wait_ready(const hx711_config_t *hx711, uint32 timeout_ms) {
    int64 start_time = esp_timer_get_time();
    int64 timeout_us = (int64)timeout_ms * 1000;

    while (gpio_get_level(hx711->dout_pin) != 0) {
        if ((esp_timer_get_time() - start_time) > timeout_us) {
            return E_NOT_OK;
        }
        TickType_t delay_ticks = pdMS_TO_TICKS(2);
        vTaskDelay(delay_ticks > 0 ? delay_ticks : 1);
    }
    return E_OK;
}

static void hx711_sort_samples(int32 *arr, uint8 n) {
    for (uint8 i = 1; i < n; i++) {
        int32 key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}
