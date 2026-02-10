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
#include "outer_loop_control.h"
#include "config.h"
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
adcs_sensor_input_t adcs_in;
adcs_output_t adcs_out;
#endif

#ifdef SIMULATION_MODE
volatile SimPacket_Input_t sim_input;
volatile SimPacket_Output_t sim_output;
volatile uint8_t sim_packet_received_main = 0;
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

  // Initialize the Outer Loop (ADCS Algorithms)
  OuterLoop_Init();

  // Start the Control Loop Timer (TIM6)
  HAL_TIM_Base_Start_IT(&htim6);

  // TEST: Request 50mA Current
  // TEST: Request 0mA Current (Zero Initial State)
  InnerLoop_SetTargetCurrent(0.0f, 0.0f, 0.0f);

#ifdef SIMULATION_MODE
  // Start receiving the first Sync Byte (State Machine)
  extern uint8_t uart_sync_byte;
  HAL_UART_Receive_IT(&huart2, &uart_sync_byte, 1);
#endif



#endif

// Global Flag for Main Context
/* sim_packet_received_main is defined in global PV section */
static float last_kbdot = -1.0f, last_kp = -1.0f, last_ki = -1.0f, last_kd = -1.0f;

/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{

#if ENABLE_INNER_LOOP_CONTROL

    // SYNC LOGIC: In Sim Mode, only run when packet arrives.
    // In Hardware Mode, run continuously (with delay).
    uint8_t run_algorithm = 0;

#ifdef SIMULATION_MODE
    if (sim_packet_received_main) {
        sim_packet_received_main = 0;
        run_algorithm = 1;
    }
#else
    run_algorithm = 1;
#endif

    if (run_algorithm) {
        data = InnerLoop_GetState();

        // 1. Prepare ADCS Input
#ifdef SIMULATION_MODE
        adcs_in.mag_field = (vec3_t){sim_input.mag_x, sim_input.mag_y, sim_input.mag_z};
        adcs_in.gyro = (vec3_t){sim_input.gyro_x, sim_input.gyro_y, sim_input.gyro_z};
        adcs_in.orientation = (quat_t){sim_input.q_w, sim_input.q_x, sim_input.q_y, sim_input.q_z};
        
        // Dynamic Gains from Sim
        if (sim_input.k_bdot != last_kbdot || sim_input.kp != last_kp || 
            sim_input.ki != last_ki || sim_input.kd != last_kd) {
            OuterLoop_SetGains(sim_input.k_bdot, sim_input.kp, sim_input.ki, sim_input.kd);
            last_kbdot = sim_input.k_bdot;
            last_kp = sim_input.kp;
            last_ki = sim_input.ki;
            last_kd = sim_input.kd;
        }
#else
        // TODO: Read from physical sensors (BNO085)
#endif

        // 2. Run ADCS Algorithms
#ifdef SIMULATION_MODE
        OuterLoop_Update(&adcs_in, &adcs_out, sim_input.dt);
#else
        OuterLoop_Update(&adcs_in, &adcs_out, 0.01f); // 100Hz default
#endif

        // 3. Command the Inner Loop
        float target_x = adcs_out.dipole_request.x * MTQ_DIPOLE_TO_AMP;
        float target_y = adcs_out.dipole_request.y * MTQ_DIPOLE_TO_AMP;
        float target_z = adcs_out.dipole_request.z * MTQ_DIPOLE_TO_AMP;
        
        InnerLoop_SetTargetCurrent(target_x, target_y, target_z);

        // 4. Telemetry (Hardware Mode Only)
#ifndef SIMULATION_MODE
        Teleplot_Update("TargetZ", target_z * 1000);
        Teleplot_Update("CurrentZ", data.measured_current_z * 1000);
        Teleplot_Update("VoltZ", data.command_voltage_z);
        HAL_Delay(50); // 20Hz Loop
#endif
    }
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
uint8_t uart_sync_byte;
uint8_t uart_payload_buffer[sizeof(SimPacket_Input_t)];
static enum { RX_SYNC1, RX_SYNC2, RX_PAYLOAD } uart_rx_state = RX_SYNC1;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    switch (uart_rx_state) {
      case RX_SYNC1:
        if (uart_sync_byte == 0xB5) {
          uart_rx_state = RX_SYNC2;
        }
        HAL_UART_Receive_IT(&huart2, &uart_sync_byte, 1);
        break;
        
      case RX_SYNC2:
        if (uart_sync_byte == 0x62) {
          uart_rx_state = RX_PAYLOAD;
          // Store header, receive rest of packet
          uart_payload_buffer[0] = 0xB5;
          uart_payload_buffer[1] = 0x62;
          HAL_UART_Receive_IT(&huart2, &uart_payload_buffer[2], sizeof(SimPacket_Input_t) - 2);
        } else {
          uart_rx_state = RX_SYNC1;
          HAL_UART_Receive_IT(&huart2, &uart_sync_byte, 1);
        }
        break;
        
      case RX_PAYLOAD:
        // Full packet received!
        memcpy((void*)&sim_input, uart_payload_buffer, sizeof(SimPacket_Input_t));
        
        InnerLoop_Update_SimDataAvailable();
        sim_packet_received_main = 1;
        
        // Reset for next packet
        uart_rx_state = RX_SYNC1;
        HAL_UART_Receive_IT(&huart2, &uart_sync_byte, 1);
        break;
    }
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
