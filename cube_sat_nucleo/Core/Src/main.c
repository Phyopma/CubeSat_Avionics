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
#include "serial_log_dma.h"
#include "imu_bno085.h"
#include "inner_loop_control.h"
#include "outer_loop_control.h"
#include "config.h"
#include "teleplot.h"
#include "app_runtime.h"
#include "task.h"
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
bno085_t imu;


volatile SimPacket_Input_t sim_input;
volatile SimPacket_Output_t sim_output;
volatile uint8_t sim_packet_received_main = 0;
volatile uint32_t sim_last_packet_ms = 0;
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
  AppRuntime_Init();

  // Start receiving the first Sync Byte (State Machine)
  extern uint8_t uart_sync_byte;
  HAL_UART_Receive_IT(&huart2, &uart_sync_byte, 1);
  AppRuntime_Start();

  /* USER CODE END 2 */

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // AppRuntime_RunOnce();
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
uint8_t uart_sync_byte;
uint8_t uart_payload_buffer[sizeof(SimPacket_Input_t)];
static enum { RX_SYNC1,
              RX_SYNC2,
              RX_PAYLOAD } uart_rx_state = RX_SYNC1;
static uint32_t sim_uart_error_count = 0U;

static void SimUart_RearmSyncRx(void)
{
  uart_rx_state = RX_SYNC1;
  (void)HAL_UART_Receive_IT(&huart2, &uart_sync_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    switch (uart_rx_state)
    {
    case RX_SYNC1:
      if (uart_sync_byte == 0xB5)
      {
        uart_rx_state = RX_SYNC2;
      }
      if (HAL_UART_Receive_IT(&huart2, &uart_sync_byte, 1) != HAL_OK)
      {
        SimUart_RearmSyncRx();
      }
      break;

    case RX_SYNC2:
      if (uart_sync_byte == 0x62)
      {
        uart_rx_state = RX_PAYLOAD;
        // Store header, receive rest of packet
        uart_payload_buffer[0] = 0xB5;
        uart_payload_buffer[1] = 0x62;
        if (HAL_UART_Receive_IT(&huart2, &uart_payload_buffer[2], sizeof(SimPacket_Input_t) - 2) != HAL_OK)
        {
          SimUart_RearmSyncRx();
        }
      }
      else
      {
        uart_rx_state = RX_SYNC1;
        if (HAL_UART_Receive_IT(&huart2, &uart_sync_byte, 1) != HAL_OK)
        {
          SimUart_RearmSyncRx();
        }
      }
      break;

    case RX_PAYLOAD:
      // Full packet received!
      memcpy((void *)&sim_input, uart_payload_buffer, sizeof(SimPacket_Input_t));

      InnerLoop_Update_SimDataAvailable();
      sim_packet_received_main = 1;
      sim_last_packet_ms = HAL_GetTick();

      // Notify ADCS task only after scheduler is running.
      if (g_adcs_task_handle != NULL &&
          xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
      {
        BaseType_t hpw = pdFALSE;
        vTaskNotifyGiveFromISR(g_adcs_task_handle, &hpw);
        portYIELD_FROM_ISR(hpw);
      }

      // Reset for next packet
      uart_rx_state = RX_SYNC1;
      if (HAL_UART_Receive_IT(&huart2, &uart_sync_byte, 1) != HAL_OK)
      {
        SimUart_RearmSyncRx();
      }
      break;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    sim_uart_error_count++;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    SimUart_RearmSyncRx();
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM6)
  {
    AppRuntime_OnControlTickFromISR();
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
