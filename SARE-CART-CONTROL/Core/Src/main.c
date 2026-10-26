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
#include "usart.h"
#include "gpio.h"

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
static void print_telemetry_report(const sim7600_gps_data_t *gps, bool gps_has_data,
                                   const nfc_card_t *nfc, bool nfc_has_card,
                                   int32_t weight_mg, bool weight_has_data)
{
  char report[768];
  int len = 0;

  /* Header & 1. GPS Reads */
  len += snprintf(report + len, sizeof(report) - len,
                  "\r\n========================================\r\n"
                  "--- GPS Reads ---\r\n");

  if (gps_has_data && (len < (int)sizeof(report)))
  {
    /* Fix & Satellites */
    len += snprintf(report + len, sizeof(report) - len,
                    "Fix Status : %s | Quality: %u\r\n"
                    "Satellites : %u in use / %u in view | HDOP: %ld.%02ld\r\n",
                    gps->valid ? "VALID (3D Fix)" : "NO FIX (Searching satellites...)",
                    (unsigned int)gps->fix_quality,
                    (unsigned int)gps->sats_used,
                    (unsigned int)gps->sats_visible,
                    (long)(gps->hdop_cd / 100),
                    (long)(gps->hdop_cd % 100));

    /* Coordinates (Fixed-point: lat_e6 / 1000000, lon_e6 / 1000000) */
    int32_t abs_lat = (gps->lat_e6 < 0) ? -gps->lat_e6 : gps->lat_e6;
    int32_t lat_int = abs_lat / 1000000;
    int32_t lat_frac = abs_lat % 1000000;

    int32_t abs_lon = (gps->lon_e6 < 0) ? -gps->lon_e6 : gps->lon_e6;
    int32_t lon_int = abs_lon / 1000000;
    int32_t lon_frac = abs_lon % 1000000;

    len += snprintf(report + len, sizeof(report) - len,
                    "Coordinates: Lat %ld.%06ld %c | Lon %ld.%06ld %c\r\n",
                    (long)lat_int, (long)lat_frac, (gps->lat_e6 >= 0) ? 'N' : 'S',
                    (long)lon_int, (long)lon_frac, (gps->lon_e6 >= 0) ? 'E' : 'W');

    /* Altitude & Speed & Course */
    int32_t abs_alt = (gps->alt_cm < 0) ? -gps->alt_cm : gps->alt_cm;
    int32_t alt_int = abs_alt / 100;
    int32_t alt_frac = abs_alt % 100;

    int32_t abs_spd = (gps->speed_ckmh < 0) ? -gps->speed_ckmh : gps->speed_ckmh;
    int32_t spd_int = abs_spd / 100;
    int32_t spd_frac = abs_spd % 100;

    len += snprintf(report + len, sizeof(report) - len,
                    "Altitude   : %ld.%02ld m | Speed: %ld.%02ld km/h | Course: %ld.%02ld deg\r\n"
                    "UTC Time   : %s | Date: %s\r\n",
                    (long)alt_int, (long)alt_frac,
                    (long)spd_int, (long)spd_frac,
                    (long)(gps->course_cdeg / 100), (long)(gps->course_cdeg % 100),
                    gps->utc_time[0] ? gps->utc_time : "--:--:--",
                    gps->utc_date[0] ? gps->utc_date : "--/--/--");
  }
  else
  {
    len += snprintf(report + len, sizeof(report) - len,
                    "Status     : WAITING FOR GPS DATA...\r\n");
  }

  /* 2. NFC Reads */
  len += snprintf(report + len, sizeof(report) - len,
                  "\r\n--- NFC Reads ---\r\n");

  if (nfc_has_card && (len < (int)sizeof(report)))
  {
    len += snprintf(report + len, sizeof(report) - len,
                    "Card Status: CARD DETECTED\r\n"
                    "UID Length : %u bytes\r\n"
                    "UID (HEX)  : ", (unsigned int)nfc->uid_len);

    for (uint8_t i = 0; i < nfc->uid_len && i < NFC_UID_MAX_LEN; i++)
    {
      len += snprintf(report + len, sizeof(report) - len,
                      "%02X%s", nfc->uid[i], (i + 1 < nfc->uid_len) ? ":" : "");
    }

    len += snprintf(report + len, sizeof(report) - len,
                    "\r\nSAK (Type) : 0x%02X | ATQA: 0x%04X\r\n",
                    nfc->sak, nfc->atqa);
  }
  else
  {
    len += snprintf(report + len, sizeof(report) - len,
                    "Card Status: NO CARD IN FIELD (Searching...)\r\n");
  }

  /* 3. Load Cell Reads */
  len += snprintf(report + len, sizeof(report) - len,
                  "\r\n--- Load Cell Reads ---\r\n");

  if (weight_has_data && (len < (int)sizeof(report)))
  {
    int32_t abs_mg = (weight_mg < 0) ? -weight_mg : weight_mg;
    len += snprintf(report + len, sizeof(report) - len,
                    "Weight     : %c%ld.%03ld g (%ld mg)\r\n"
                    "Status     : OK (Filtered)\r\n",
                    (weight_mg < 0) ? '-' : ' ',
                    (long)(abs_mg / 1000),
                    (long)(abs_mg % 1000),
                    (long)weight_mg);
  }
  else
  {
    len += snprintf(report + len, sizeof(report) - len,
                    "Status     : WAITING FOR LOAD CELL SAMPLES...\r\n");
  }

  len += snprintf(report + len, sizeof(report) - len,
                  "========================================\r\n");

  /* Single atomic transmit: eliminates interleaving & reduces CPU blocking */
  if (len > 0)
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)report, (uint16_t)len, 500);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
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
  /* Initialize ECUAL layer (SIM7600 GPS on USART1, NFC on SPI1, HX711 on PB0/PB1) */
  if (E_OK != ecual_init())
  {
    char boot_msg[128];
    snprintf(boot_msg, sizeof(boot_msg),
             "ECUAL Init Warning: GPS=%s, NFC=%s, HX711=%s\r\n",
             ecual_gps_is_ready() ? "OK" : "FAIL",
             ecual_nfc_is_ready() ? "OK" : "FAIL",
             ecual_hx711_is_ready() ? "OK" : "FAIL");
    HAL_UART_Transmit(&huart2, (uint8_t *)boot_msg, (uint16_t)strlen(boot_msg), 100);
  }
  else
  {
    const char ok_msg[] = "ECUAL Init OK: GPS, NFC & Load Cell modules online.\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)ok_msg, (uint16_t)strlen(ok_msg), 100);
  }

  /* Zero the load cell (cart must be empty at boot) */
  if (ecual_hx711_is_ready())
  {
    if (E_OK != ecual_hx711_tare(10U))
    {
      const char tare_err[] = "HX711 tare FAILED! Check load cell wiring.\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)tare_err, (uint16_t)strlen(tare_err), 100);
    }
    else
    {
      const char tare_ok[] = "HX711 tare OK: Load cell zeroed successfully.\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)tare_ok, (uint16_t)strlen(tare_ok), 100);
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

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Service initialized components through the ECUAL gateway (non-blocking). */
    ecual_task();

    /* Periodic report over UART2 every 1000 ms */
    if ((HAL_GetTick() - last_report_tick) >= 1000U)
    {
      last_report_tick = HAL_GetTick();

      gps_ok = (E_OK == ecual_gps_get_data(&gps_data));
      nfc_ok = ecual_nfc_get_card(&nfc_card);
      weight_ok = ecual_hx711_get_weight_mg(&weight_mg);

      print_telemetry_report(&gps_data, gps_ok, &nfc_card, nfc_ok, weight_mg, weight_ok);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
