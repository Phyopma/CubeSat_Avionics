/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adt7420.h"
#include "serial_log_dma.h"
#include "imu_bno085.h"
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  serial_log_init(&huart2);
  const char hello[] = "UART DMA logger online\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)hello, sizeof(hello)-1, 100);

  // ADT7420 begin
//  log_printf_dma("ADT7420 bring-up\n");
//
//  ADT7420_Handle t;
//  uint8_t addr7 = 0x4B; // set to match JP2/JP1
//
//
//  if (ADT7420_Init(&t, &hi2c1, addr7) != HAL_OK) {
//	  //		printf("ADT7420 init failed (address 0x%02X)\r\n", addr7);
//	  log_printf_dma("Init failed at 0x%02X\n", addr7);
//	  Error_Handler();
//  }
//
//  uint8_t id=0, cfg=0;
//  ADT7420_ReadID(&t, &id);
//  ADT7420_ReadConfig(&t, &cfg);
  //  log_printf_dma("ID=0x%02X (expect 0xCB), CONFIG=0x%02X\n", id, cfg);
  // ADT7420_Set16Bit(&t, 0); // uncomment to force 13-bit
  // ADT7420 end

  // IMU Begin
  bno085_t g_imu;
  if (!BNO085_Begin(&g_imu)) {
	  log_printf_dma("BNO085 init failed\r\n");
  } else {
	  log_printf_dma("BNO085 init OK\r\n");
  }

  uint32_t last_print = HAL_GetTick();

  // IMU End

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
// ADT7420 begin
//	float c = 0.0f;
//	if (ADT7420_ReadCelsius(&t, &c) == HAL_OK) {
//	  log_printf_dma("Temp = %.3f C", c);
//	} else {
//	  log_printf_dma("Temp read error");
//	}
//	HAL_Delay(100);
//	ADT7420 end

//	IMU begin
	(void)BNO085_Service(&g_imu);

	bno085_quat_t q;
	if (BNO085_GetQuaternion(&g_imu, &q)) {
		// Print on arrival as well
		log_printf_dma("Q: %.6f, %.6f, %.6f, %.6f ts=%lu\r\n",
			   q.i, q.j, q.k, q.real, (unsigned long)q.timestamp_us);
	}

	if ((HAL_GetTick() - last_print) >= 100) {
		last_print += 100;
		// Optional periodic print if no new data
	}

//	IMU end
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

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
