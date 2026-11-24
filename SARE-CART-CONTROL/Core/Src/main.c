/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static volatile uint32_t s_total_button_presses = 0;
static volatile uint16_t s_button_taps_in_window = 0;
static const char *s_last_button_event_str = "NONE";

static void on_button_event(struct button_s *btn, button_event_t event, void *user_data) {
  (void)btn;
  (void)user_data;

  switch (event) {
  case BUTTON_EVENT_PRESSED:
    s_total_button_presses++;
    s_button_taps_in_window++;
    s_last_button_event_str = "PRESSED";
    break;

  case BUTTON_EVENT_RELEASED:
    s_last_button_event_str = "RELEASED";
    break;

  case BUTTON_EVENT_CLICK:
    s_last_button_event_str = "CLICK";
    break;

  case BUTTON_EVENT_DOUBLE_CLICK:
    s_last_button_event_str = "DOUBLE_CLICK";
    break;

  case BUTTON_EVENT_LONG_PRESS:
    s_last_button_event_str = "LONG_PRESS";
    break;

  default:
    break;
  }
}

static void print_telemetry_report(const sim7600_gps_data_t *gps,
                                   bool gps_has_data,
                                   const sim7600_gps_diag_t *gps_diag,
                                   const nfc_card_t *nfc, bool nfc_has_card,
                                   int32_t weight_mg, bool weight_has_data,
                                   bool weight_is_stable, int32_t raw_counts,
                                   int32_t tare_offset) {
  char report[1024];
  int len = 0;

  /* Header & 1. GPS Reads */
  len += snprintf(report + len, sizeof(report) - len,
                  "\r\n========================================\r\n"
                  "--- GPS Reads ---\r\n");

  if (gps_has_data && (len < (int)sizeof(report))) {
    /* Fix & Satellites */
    len += snprintf(
        report + len, sizeof(report) - len,
        "Fix Status : %s | Quality: %u\r\n"
        "Satellites : %u in use / %u in view | HDOP: %ld.%02ld\r\n",
        gps->valid ? "VALID (3D Fix)" : "NO FIX (Searching satellites...)",
        (unsigned int)gps->fix_quality, (unsigned int)gps->sats_used,
        (unsigned int)gps->sats_visible, (long)(gps->hdop_cd / 100),
        (long)(gps->hdop_cd % 100));

    /* Coordinates (Fixed-point: lat_e6 / 1000000, lon_e6 / 1000000) */
    int32_t abs_lat = (gps->lat_e6 < 0) ? -gps->lat_e6 : gps->lat_e6;
    int32_t lat_int = abs_lat / 1000000;
    int32_t lat_frac = abs_lat % 1000000;

    int32_t abs_lon = (gps->lon_e6 < 0) ? -gps->lon_e6 : gps->lon_e6;
    int32_t lon_int = abs_lon / 1000000;
    int32_t lon_frac = abs_lon % 1000000;

    len +=
        snprintf(report + len, sizeof(report) - len,
                 "Coordinates: Lat %ld.%06ld %c | Lon %ld.%06ld %c\r\n",
                 (long)lat_int, (long)lat_frac, (gps->lat_e6 >= 0) ? 'N' : 'S',
                 (long)lon_int, (long)lon_frac, (gps->lon_e6 >= 0) ? 'E' : 'W');

    /* Altitude & Speed & Course */
    int32_t abs_alt = (gps->alt_cm < 0) ? -gps->alt_cm : gps->alt_cm;
    int32_t alt_int = abs_alt / 100;
    int32_t alt_frac = abs_alt % 100;

    int32_t abs_spd =
        (gps->speed_ckmh < 0) ? -gps->speed_ckmh : gps->speed_ckmh;
    int32_t spd_int = abs_spd / 100;
    int32_t spd_frac = abs_spd % 100;

    len +=
        snprintf(report + len, sizeof(report) - len,
                 "Altitude   : %ld.%02ld m | Speed: %ld.%02ld km/h | Course: "
                 "%ld.%02ld deg\r\n"
                 "UTC Time   : %s | Date: %s\r\n",
                 (long)alt_int, (long)alt_frac, (long)spd_int, (long)spd_frac,
                 (long)(gps->course_cdeg / 100), (long)(gps->course_cdeg % 100),
                 gps->utc_time[0] ? gps->utc_time : "--:--:--",
                 gps->utc_date[0] ? gps->utc_date : "--/--/--");
  } else {
    len += snprintf(report + len, sizeof(report) - len,
                    "Status     : WAITING FOR GPS DATA...\r\n");
  }

  /* Diagnostics line */
  if (gps_diag && (len < (int)sizeof(report))) {
    len += snprintf(report + len, sizeof(report) - len,
                    "GPS Diag   : UART RX: %lu B | NMEA Pkts: %lu | ChkErr: "
                    "%lu | FrameErr: %lu\r\n"
                    "Last Raw   : \"%.55s\"\r\n",
                    (unsigned long)gps_diag->rx_bytes,
                    (unsigned long)gps_diag->nmea_sentences,
                    (unsigned long)gps_diag->nmea_bad_checksum,
                    (unsigned long)gps_diag->hw_framing_errors,
                    gps_diag->last_line[0] ? gps_diag->last_line
                                           : "(none - check wiring/power)");
  }

  /* 2. NFC Reads */
  len +=
      snprintf(report + len, sizeof(report) - len, "\r\n--- NFC Reads ---\r\n");

  if (!ecual_nfc_is_ready()) {
    nfc_error_t nfc_err = NFC_ERR_NONE;
    (void)ecual_nfc_get_error(&nfc_err);
    const char *err_str = "TIMEOUT";
    switch (nfc_err) {
      case NFC_ERR_TIMEOUT: err_str = "TIMEOUT (No Response on SPI)"; break;
      case NFC_ERR_NO_ACK:  err_str = "NO ACK"; break;
      case NFC_ERR_FRAME:   err_str = "BAD FRAME / WRONG IC"; break;
      case NFC_ERR_BUS:     err_str = "SPI BUS ERROR"; break;
      default:              err_str = "INIT FAILED"; break;
    }
    len += snprintf(report + len, sizeof(report) - len,
                    "Hardware   : OFFLINE [Error: %s]\r\n"
                    "Hint       : Check SPI DIP Switch (SEL0=0, SEL1=1) & Wiring\r\n",
                    err_str);
  } else if (nfc_has_card && (len < (int)sizeof(report))) {
    len += snprintf(report + len, sizeof(report) - len,
                    "Card Status: CARD DETECTED\r\n"
                    "UID Length : %u bytes\r\n"
                    "UID (HEX)  : ",
                    (unsigned int)nfc->uid_len);

    for (uint8_t i = 0; i < nfc->uid_len && i < NFC_UID_MAX_LEN; i++) {
      len += snprintf(report + len, sizeof(report) - len, "%02X%s", nfc->uid[i],
                      (i + 1 < nfc->uid_len) ? ":" : "");
    }

    len += snprintf(report + len, sizeof(report) - len,
                    "\r\nSAK (Type) : 0x%02X | ATQA: 0x%04X\r\n", nfc->sak,
                    nfc->atqa);
  } else {
    len += snprintf(report + len, sizeof(report) - len,
                    "Card Status: NO CARD IN FIELD (Searching...)\r\n");
  }

  /* 3. Load Cell Reads */
  len += snprintf(report + len, sizeof(report) - len,
                  "\r\n--- Load Cell Reads ---\r\n");

  if (!ecual_hx711_is_ready()) {
    len += snprintf(report + len, sizeof(report) - len,
                    "Status     : OFFLINE (Not Connected on PB0/PB1)\r\n");
  } else if (weight_has_data && (len < (int)sizeof(report))) {
    if (weight_mg < 0) {
      weight_mg = 0;
    }
    int32_t net_raw = raw_counts - tare_offset;
    len += snprintf(report + len, sizeof(report) - len,
                    "Weight     : %ld.%03ld g (%ld mg)\r\n"
                    "RAW Counts : %ld | Offset: %ld | Net RAW: %ld\r\n"
                    "Status     : %s\r\n",
                    (long)(weight_mg / 1000), (long)(weight_mg % 1000),
                    (long)weight_mg, (long)raw_counts, (long)tare_offset,
                    (long)net_raw,
                    weight_is_stable ? "STABLE (Anti-Creep Active)"
                                     : "SETTLING (Tracking...)");
  } else {
    uint8_t dout_pin_val = (GPIOB->IDR & GPIO_PIN_1) ? 1 : 0;
    len += snprintf(report + len, sizeof(report) - len,
                    "Status     : WAITING FOR LOAD CELL SAMPLES...\r\n"
                    "Pin Diag   : DOUT (PB1) = %u (%s) | SCK (PB0) = %u\r\n",
                    dout_pin_val,
                    (dout_pin_val == 0) ? "PULSING / READY"
                                        : "HIGH (Check DT/SCK wiring & Power)",
                    (GPIOB->ODR & GPIO_PIN_0) ? 1 : 0);
  }

  /* 4. Actuators Reads & Status */
  rgb_color_val_t rgb_val = {0};
  (void)ecual_rgb_get_rgb(&rgb_val);
  const char *color_name = "OFF";
  if (rgb_val.r == 255 && rgb_val.g == 0 && rgb_val.b == 0) {
    color_name = "RED (PB7)";
  } else if (rgb_val.r == 0 && rgb_val.g == 255 && rgb_val.b == 0) {
    color_name = "GREEN (PB8)";
  } else if (rgb_val.r == 0 && rgb_val.g == 0 && rgb_val.b == 255) {
    color_name = "BLUE (PB9)";
  } else if (rgb_val.r == 255 && rgb_val.g == 255 && rgb_val.b == 255) {
    color_name = "WHITE (PB7+PB8+PB9)";
  } else if (rgb_val.r > 0 || rgb_val.g > 0 || rgb_val.b > 0) {
    color_name = "ACTIVE";
  }

  bool currently_pressed = ecual_button_is_pressed();
  uint16_t window_taps = s_button_taps_in_window;
  s_button_taps_in_window = 0; /* Clear accumulator for next periodic frame */

  const char *btn_state_str;
  if (currently_pressed) {
    btn_state_str = "HELD DOWN (LOW)";
  } else if (window_taps > 0) {
    btn_state_str = "CLICKED (RELEASED)";
  } else {
    btn_state_str = "RELEASED (HIGH)";
  }

  len += snprintf(report + len, sizeof(report) - len,
                  "\r\n--- Actuators Reads ---\r\n"
                  "RGB LED    : %s [R:%u, G:%u, B:%u]\r\n"
                  "Buzzer     : %s (Pin PB10)\r\n"
                  "Button     : Status: %s | Taps in Frame: %u | Total Presses: %lu | Last Event: %s\r\n",
                  color_name, (unsigned int)rgb_val.r, (unsigned int)rgb_val.g,
                  (unsigned int)rgb_val.b,
                  ecual_buzzer_is_busy() ? "BEEPING / ACTIVE" : "IDLE",
                  btn_state_str,
                  (unsigned int)window_taps,
                  (unsigned long)s_total_button_presses,
                  s_last_button_event_str);

  len += snprintf(report + len, sizeof(report) - len,
                  "========================================\r\n");

  /* Single atomic transmit: eliminates interleaving & reduces CPU blocking */
  if (len > 0) {
    HAL_UART_Transmit(&huart2, (uint8_t *)report, (uint16_t)len, 200);
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize ECUAL layer (GPS, NFC, HX711, RGB LED, Buzzer, Button) */
  if (E_OK != ecual_init()) {
    char boot_msg[160];
    snprintf(boot_msg, sizeof(boot_msg),
             "ECUAL Init: GPS=%s, NFC=%s, HX711=%s, RGB=%s, BZ=%s, BTN=%s\r\n",
             ecual_gps_is_ready() ? "OK" : "FAIL",
             ecual_nfc_is_ready() ? "OK" : "FAIL",
             ecual_hx711_is_ready() ? "OK" : "FAIL",
             ecual_rgb_led_is_ready() ? "OK" : "FAIL",
             ecual_buzzer_is_ready() ? "OK" : "FAIL",
             ecual_button_is_ready() ? "OK" : "FAIL");
    HAL_UART_Transmit(&huart2, (uint8_t *)boot_msg, (uint16_t)strlen(boot_msg),
                      100);
  } else {
    const char ok_msg[] =
        "ECUAL Init OK: All 6 Modules Online (GPS, NFC, Load Cell, RGB, Buzzer, Button).\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)ok_msg, (uint16_t)strlen(ok_msg),
                      100);
  }

  /* Register button interactive event callback */
  ecual_button_set_callback(on_button_event, NULL);

  /* Wait 400ms for HX711 analog bandgap & power supply to settle before taring
   */
  HAL_Delay(400);

  /* Zero the load cell: 8 samples at boot (~800ms) */
  if (ecual_hx711_is_ready()) {
    if (E_OK != ecual_hx711_tare(8U)) {
      const char tare_err[] = "HX711 tare FAILED! Check load cell wiring.\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)tare_err,
                        (uint16_t)strlen(tare_err), 100);
    } else {
      const char tare_ok[] = "HX711 tare OK.\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)tare_ok, (uint16_t)strlen(tare_ok),
                        100);
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_report_tick = 0;
  sim7600_gps_data_t gps_data;
  nfc_card_t nfc_card;
  int32_t weight_mg = 0;
  bool gps_ok = false;
  bool nfc_ok = false;
  bool weight_ok = false;

  memset(&gps_data, 0, sizeof(gps_data));
  memset(&nfc_card, 0, sizeof(nfc_card));

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Service initialized components through the ECUAL gateway (non-blocking). */
    ecual_task();

    /* Actuator Test Sequence (every 1500 ms):
     * Step 0: RED   (PB7)           + 50ms Beep (PB10)
     * Step 1: GREEN (PB8)           + 50ms Beep (PB10)
     * Step 2: BLUE  (PB9)           + 50ms Beep (PB10)
     * Step 3: WHITE (PB7, PB8, PB9) + 50ms Beep (PB10)
     * Step 4: OFF
     */
    static uint32_t last_actuator_test_tick = 0;
    static uint8_t actuator_test_step = 0;
    if ((HAL_GetTick() - last_actuator_test_tick) >= 1500U) {
      last_actuator_test_tick = HAL_GetTick();

      switch (actuator_test_step) {
      case 0:
        (void)ecual_rgb_set_color(RGB_COLOR_RED);
        (void)ecual_buzzer_beep(50U);
        break;
      case 1:
        (void)ecual_rgb_set_color(RGB_COLOR_GREEN);
        (void)ecual_buzzer_beep(50U);
        break;
      case 2:
        (void)ecual_rgb_set_color(RGB_COLOR_BLUE);
        (void)ecual_buzzer_beep(50U);
        break;
      case 3:
        (void)ecual_rgb_set_color(RGB_COLOR_WHITE);
        (void)ecual_buzzer_beep(50U);
        break;
      case 4:
      default:
        (void)ecual_rgb_turn_off();
        break;
      }

      actuator_test_step = (actuator_test_step + 1) % 5;
    }

    /* Periodic report over UART2 every 500 ms */
    if ((HAL_GetTick() - last_report_tick) >= 500U) {
      last_report_tick = HAL_GetTick();

      sim7600_gps_diag_t gps_diag;
      (void)ecual_gps_get_diag(&gps_diag);
      gps_ok = (E_OK == ecual_gps_get_data(&gps_data));
      nfc_ok = ecual_nfc_get_card(&nfc_card);
      weight_ok = ecual_hx711_get_weight_mg(&weight_mg);
      bool weight_stable = ecual_hx711_is_stable();

      int32_t raw_counts = 0;
      int32_t tare_offset = 0;
      (void)ecual_hx711_get_raw(&raw_counts);
      (void)ecual_hx711_get_offset(&tare_offset);

      print_telemetry_report(&gps_data, gps_ok, &gps_diag, &nfc_card, nfc_ok,
                             weight_mg, weight_ok, weight_stable, raw_counts,
                             tare_offset);
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(
      PWR_REGULATOR_VOLTAGE_SCALE1); /* Required for 84MHz */

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;  /* HSI=16MHz / 8 = 2MHz VCO input  */
  RCC_OscInitStruct.PLL.PLLN = 84; /* × 84 = 168MHz VCO               */
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; /* 168/2 = 84MHz SYSCLK   */
  RCC_OscInitStruct.PLL.PLLQ = 4; /* USB/SDIO (unused here, any value) */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; /* HCLK  = 84MHz */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; /* APB1  = 42MHz (max 42) */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; /* APB2  = 84MHz (max 84) */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
