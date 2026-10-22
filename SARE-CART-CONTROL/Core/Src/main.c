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
static void print_gps_data(const sim7600_gps_data_t *gps)
{
  char buffer[128];

  /* Header */
  HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n--- GPS REPORT ---\r\n", 22, 100);

  /* Fix Status & Satellites */
  snprintf(buffer, sizeof(buffer), "Fix: %s | Q: %u | Sats: %u (in use) / %u (in view) | HDOP: %ld.%02ld\r\n",
           gps->valid ? "VALID" : "NO FIX",
           (unsigned int)gps->fix_quality,
           (unsigned int)gps->sats_used,
           (unsigned int)gps->sats_visible,
           (long)(gps->hdop_cd / 100),
           (long)(gps->hdop_cd % 100));
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)strlen(buffer), 100);

  /* Coordinates (Fixed-point: lat_e6 / 1000000, lon_e6 / 1000000) */
  int32_t lat_int = gps->lat_e6 / 1000000;
  int32_t lat_frac = gps->lat_e6 % 1000000;
  if (lat_frac < 0) lat_frac = -lat_frac;

  int32_t lon_int = gps->lon_e6 / 1000000;
  int32_t lon_frac = gps->lon_e6 % 1000000;
  if (lon_frac < 0) lon_frac = -lon_frac;

  snprintf(buffer, sizeof(buffer), "Lat: %ld.%06ld %c | Lon: %ld.%06ld %c\r\n",
           (long)lat_int, (long)lat_frac, (gps->lat_e6 >= 0) ? 'N' : 'S',
           (long)lon_int, (long)lon_frac, (gps->lon_e6 >= 0) ? 'E' : 'W');
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)strlen(buffer), 100);

  /* Altitude & Speed */
  int32_t alt_int = gps->alt_cm / 100;
  int32_t alt_frac = gps->alt_cm % 100;
  if (alt_frac < 0) alt_frac = -alt_frac;

  int32_t spd_int = gps->speed_ckmh / 100;
  int32_t spd_frac = gps->speed_ckmh % 100;
  if (spd_frac < 0) spd_frac = -spd_frac;

  snprintf(buffer, sizeof(buffer), "Alt: %ld.%02ld m | Speed: %ld.%02ld km/h | Course: %ld.%02ld deg\r\n",
           (long)alt_int, (long)alt_frac,
           (long)spd_int, (long)spd_frac,
           (long)(gps->course_cdeg / 100), (long)(gps->course_cdeg % 100));
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)strlen(buffer), 100);

  /* UTC Time & Date */
  snprintf(buffer, sizeof(buffer), "UTC Time: %s | Date: %s\r\n",
           gps->utc_time[0] ? gps->utc_time : "--:--:--",
           gps->utc_date[0] ? gps->utc_date : "--/--/--");
  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)strlen(buffer), 100);

  HAL_UART_Transmit(&huart2, (uint8_t *)"--------------------\r\n", 22, 100);
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
  /* Initialize ECUAL layer (SIM7600 GPS module on USART1) */
  if (E_OK != ecual_init())
  {
    const char err_msg[] = "ECUAL GPS Init FAILED! Check module power/wiring.\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)err_msg, (uint16_t)strlen(err_msg), 100);
  }
  else
  {
    const char ok_msg[] = "ECUAL GPS Init OK. Starting standard sequence...\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)ok_msg, (uint16_t)strlen(ok_msg), 100);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_gps_report_tick = 0;
  sim7600_gps_data_t gps_data;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 1) Run GPS service task (non-blocking: assembles NMEA sentences and parses them) */
    sim7600_gps_task();

    /* 2) Periodic report over UART2 every 1000 ms */
    if ((HAL_GetTick() - last_gps_report_tick) >= 1000U)
    {
      last_gps_report_tick = HAL_GetTick();

      if (E_OK == sim7600_gps_get_data(&gps_data))
      {
        print_gps_data(&gps_data);
      }
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
