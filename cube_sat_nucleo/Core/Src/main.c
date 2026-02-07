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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adt7420.h"
#include "serial_log_dma.h"
#include "imu_bno085.h"
#include "inner_loop_control.h"
#include "teleplot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ENABLE_INNER_LOOP_CONTROL 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bno085_t imu;




#if ENABLE_INNER_LOOP_CONTROL
mtq_state_t data;
#endif

#ifdef SIMULATION_MODE
volatile SimPacket_Input_t sim_input;
volatile SimPacket_Output_t sim_output;
#endif
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
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
#ifndef SIMULATION_MODE
  serial_log_init(&huart2);
  log_printf_dma("UART DMA logger online\r\n");
#endif

  // ADT7420 begin


  // IMU Begin


#if ENABLE_INNER_LOOP_CONTROL
  // Initialize the Inner Loop (Starts Drivers & Math)
  InnerLoop_Init();

  // Start the Control Loop Timer (TIM6)
  HAL_TIM_Base_Start_IT(&htim6);

  // TEST: Request 50mA Current
  InnerLoop_SetTargetCurrent(0.03f);

#ifdef SIMULATION_MODE
  // Start receiving the first Simulation Packet
  HAL_UART_Receive_IT(&huart2, (uint8_t*)&sim_input, sizeof(sim_input));
#endif


#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


#if ENABLE_INNER_LOOP_CONTROL
    // // 1. Prepare Data
    // InnerLoop_PrintTelemetry(msg_buffer);

    // // 2. Send via DMA (Non-blocking)
    // HAL_UART_Transmit_DMA(&huart2, (uint8_t *)msg_buffer, strlen(msg_buffer));

    data = InnerLoop_GetState();

    // 2. Send to Teleplot
#ifndef SIMULATION_MODE
	// Plot the Target Current (Blue line)
	Teleplot_Update("Target", data.target_current * 1000);

	// Plot Measured Current (Green line - Essential for Tuning!)
	Teleplot_Update("Current", data.measured_current * 1000);

	Teleplot_Update("Error", (data.target_current - data.measured_current) * 1000);

	// Plot the Voltage being applied (Red line)
	Teleplot_Update("Voltage", data.command_voltage);
    HAL_Delay(50);
#endif // !SIMULATION_MODE



//    // 1. MANUALLY Force 5.0V output (Full Power)
//        // This bypasses the PI controller completely.
//        HBridge_SetVoltage(voltage, 5.0f);
//
//        // 2. Read the sensor directly
//
//        float raw_amps = CurrentSensor_Read_Amps();
//
//        // 3. Teleplot Debugging
//        // "ForceVolts" should show 5.0
//        Teleplot_Update("ForceVolts", voltage);
//        // "SensAmps" will show the real sensor reading
//        Teleplot_Update("SensAmps", raw_amps);
//
//        // 4. Console Log for sanity (Check Serial Terminal if Teleplot is confusing)
//        if (raw_amps == 0.0f) {
//            // If this prints, your sensor is either broken, address is wrong, or shunt is blown.
//            // Or your battery is unplugged.
//            // log_printf_dma("Reading 0.0 Amps...\r\n");
//        }
//
//        HAL_Delay(100);


#endif
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#if ENABLE_INNER_LOOP_CONTROL
// Timer 6 Interrupt - Runs exactly every 1ms (1kHz)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    InnerLoop_Update();
  }
}
#endif

#ifdef SIMULATION_MODE
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    // 1. We just received new inputs in `sim_input`. 
    // The Timer Interrupt (1kHz) will pick this up automatically.
    
    // 2. Restart Reception for the next packet
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&sim_input, sizeof(sim_input));
  }
}
#endif

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
