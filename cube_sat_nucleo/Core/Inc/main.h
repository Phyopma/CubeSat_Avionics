/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MTQ_SLEEP_Pin GPIO_PIN_4
#define MTQ_SLEEP_GPIO_Port GPIOA
#define MTQ_FAULT_Pin GPIO_PIN_5
#define MTQ_FAULT_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_2
#define IMU_CS_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_10
#define IMU_INT_GPIO_Port GPIOA
#define IMU_RST_Pin GPIO_PIN_5
#define IMU_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SIMULATION_MODE 1
// #define SIMULATION_MODE 0 // Comment out line above to disable

#ifdef SIMULATION_MODE
// 1 byte alignment to ensure Python struct.pack() matches exactly
typedef struct __attribute__((packed)) {
    uint16_t header;    // 0xB5 0x62 Sync
    float current_amps_x;
    float current_amps_y;
    float current_amps_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    float q_w, q_x, q_y, q_z;
    float k_bdot;       // B-Dot Gain
    float kp;           // Pointing Proportional Gain
    float ki;           // Pointing Integral Gain
    float kd;           // Pointing Derivative Gain
    float dt;           // Simulation Step Size
    uint8_t debug_flags; // Bit0: Open Loop, Bits1-2: Forced Mode, Bit3: Reset Controller State (edge-triggered)
    uint16_t max_voltage_mV;       // Runtime voltage clamp override [mV]
    uint16_t dipole_strength_milli; // Runtime dipole strength override [milli Am^2/A]
} SimPacket_Input_t;

typedef struct __attribute__((packed)) {
    uint16_t header;    // 0xB5 0x62 (Synchronization Word)
    float command_voltage_x;
    float command_voltage_y;
    float command_voltage_z;
    uint8_t adcs_mode;  // Packed telemetry byte:
                        // bits0-1 mode (0:Idle,1:Detumble,2:Spin,3:Pointing),
                        // bit2 integral clamp/freeze flag,
                        // bits3-7 projection-loss quantized [0..31]
    int16_t m_cmd_q15_x;
    int16_t m_cmd_q15_y;
    int16_t m_cmd_q15_z;
    int16_t tau_raw_q15_x;
    int16_t tau_raw_q15_y;
    int16_t tau_raw_q15_z;
    int16_t tau_proj_q15_x;
    int16_t tau_proj_q15_y;
    int16_t tau_proj_q15_z;
    uint8_t telemetry_flags; // bits0-3: packet version, bit4: m saturation,
                             // bit5: tau_raw saturation, bit6: tau_proj saturation
} SimPacket_Output_t;

extern volatile SimPacket_Input_t sim_input;
extern volatile SimPacket_Output_t sim_output;
extern UART_HandleTypeDef huart2; // Ensure this is visible if needed
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
