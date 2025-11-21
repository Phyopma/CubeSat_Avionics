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
bno085_t imu;

uint32_t last_print_time = 0;
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
  log_printf_dma("UART DMA logger online\r\n");

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


    BNO085_Log("System Booting...\r\n");

    // 1. Initialize the Sensor
    if (!BNO085_Begin(&imu)) {
        BNO085_Log("BNO085 Init FAILED. Halting.\r\n");
        while(1);
    }
    BNO085_Log("BNO085 Initialized.\r\n");

    //    10000us = 10ms = 100Hz

    // Standard 9-axis Fusion (North relative to Earth)
//    BNO085_EnableRotationVector(&imu, 10000);

    // 6-axis Fusion (North relative to startup, NO MAGNETOMETER)
    // Best for CubeSat spinning if magnetorquers interfere
//    BNO085_EnableGameRotationVector(&imu, 10000);

    // Calibrated Gyroscope (rad/s) - Essential for B-Dot control
//    BNO085_EnableGyroscope(&imu, 10000);

    // Calibrated Magnetometer (uTesla) - Essential for B-Dot control
//    BNO085_EnableMagnetometer(&imu, 20000); // 50Hz is usually enough for mag

    // Linear Acceleration (m/s^2) - Gravity removed
    BNO085_EnableLinearAccelerometer(&imu, 10000);

    BNO085_Log("Sensors Enabled. Starting Loop...\r\n");
    uint32_t last_print_time = 0;

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

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Get the current time at the start of the loop
//	  uint32_t current_time = HAL_GetTick();
	  BNO085_Service(&imu, NULL);


	  if (HAL_GetTick() - last_print_time > 200)
	      {
	          last_print_time = HAL_GetTick();

	      // --- A. Rotation Vector (Quat) ---
//	      bno085_quat_t q;
//	      if (BNO085_GetQuaternion(&imu, &q)) {
//	          // Log: [RV] R:1.000 I:0.000 J:0.000 K:0.000 (Acc: 0.1)
//	          BNO085_Log("[RV] R:%.3f I:%.3f J:%.3f K:%.3f (Acc: %.2f)\r\n",
//	                     q.real, q.i, q.j, q.k, q.accuracy_rad);
//	      }

	      // --- B. Game Rotation Vector (Quat - No Mag) ---
//	      if (BNO085_GetGameQuaternion(&imu, &q)) {
//	          // Log: [GM] R:1.000 I:0.000 ...
//	          BNO085_Log("[GM] R:%.3f I:%.3f J:%.3f K:%.3f\r\n",
//	                     q.real, q.i, q.j, q.k);
//	      }

	      // --- C. Gyroscope (rad/s) ---
//	      bno085_vec3_t g;
//	      if (BNO085_GetGyroscope(&imu, &g)) {
//	          // Log: [GY] X:0.01 Y:-0.02 Z:0.00
//	          BNO085_Log("[GY] X:%.3f Y:%.3f Z:%.3f\r\n", g.x, g.y, g.z);
//	      }

	      // --- D. Magnetometer (uTesla) ---
//	      bno085_vec3_t m;
//	      if (BNO085_GetMagnetometer(&imu, &m)) {
//	          // Log: [MG] X:20.5 Y:10.1 Z:-40.2
//	          BNO085_Log("[MG] X:%.1f Y:%.1f Z:%.1f\r\n", m.x, m.y, m.z);
//	      }

	      // --- E. Linear Acceleration (m/s^2) ---
	      bno085_vec3_t a;
	      if (BNO085_GetLinearAcceleration(&imu, &a)) {
	          // Log: [LA] X:0.1 Y:0.0 Z:9.8
	          BNO085_Log("[LA] X:%.2f Y:%.2f Z:%.2f\r\n", a.x, a.y, a.z);
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
