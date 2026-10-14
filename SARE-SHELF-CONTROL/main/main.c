#include "main.h"
#include "esp_log.h"
// #include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SHELF_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "=============================================");
    ESP_LOGI(TAG, "   SARE Smart Shelf Weight Monitoring System ");
    ESP_LOGI(TAG, "=============================================");

    /* Initialize ECUAL Drivers (HX711 + 50Kg Load Cell) */
    if (E_OK != ecual_init()) {
        ESP_LOGW(TAG, "ECUAL initialization encountered issues, checking sensor state...");
    } else {
        ESP_LOGI(TAG, "ECUAL initialization completed successfully.");
    }

    ESP_LOGI(TAG, "Load Cell Config: DOUT=GPIO%d, SCK=GPIO%d", loadcell.dout_pin, loadcell.sck_pin);
    ESP_LOGI(TAG, "Tare Baseline Offset: %ld", (long)loadcell.offset);
    ESP_LOGI(TAG, "Scale Factor: %.2f counts/kg", loadcell.scale);
    ESP_LOGI(TAG, "Max Rated Capacity: %.1f kg (Overload margin: %.1f kg)",
             loadcell.max_capacity_kg, loadcell.overload_margin_kg);
    ESP_LOGI(TAG, "Starting periodic weight measurements...\n");

    while (1) {
        int32 raw_val = 0;
        float weight_kg = 0.0f;
        float weight_g = 0.0f;
        bool is_overloaded = false;

        /* Check overload condition first */
        if (E_OK == ecual_hx711_is_overloaded(&loadcell, 10, &is_overloaded) && is_overloaded) {
            ecual_hx711_get_units(&loadcell, 10, &weight_kg);
            ESP_LOGE(TAG, "[WARNING] OVERLOAD DETECTED! Current Load: %.2f kg exceeds 50.0 kg limit!", weight_kg);
        } else {
            /* Read weight in kilograms and grams */
            Std_ReturnType status_kg = ecual_hx711_get_weight_kg(&loadcell, 10, &weight_kg);
            ecual_hx711_get_weight_g(&loadcell, 10, &weight_g);
            ecual_hx711_read_raw(&loadcell, &raw_val);

            if (E_OK == status_kg) {
                ESP_LOGI(TAG, "Shelf Weight: %.3f kg (%.1f g) | Raw ADC: %ld",
                         weight_kg, weight_g, (long)raw_val);
            } else {
                ESP_LOGW(TAG, "Sensor read anomaly or weight out of bounds (Weight: %.3f kg, Raw: %ld)",
                         weight_kg, (long)raw_val);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}