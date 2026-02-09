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
    float kd;           // Pointing Derivative Gain
    float target_current_cmd; // Keeping this for now as generic command or debug
} SimPacket_Input_t;

typedef struct __attribute__((packed)) {
    uint16_t header;    // 0xB5 0x62 (Synchronization Word)
    float command_voltage_x;
    float command_voltage_y;
    float command_voltage_z;
    uint8_t adcs_mode;  // 0:Idle, 1:Detumble, 2:Spin, 3:Pointing
    uint8_t padding[3]; // Align to 4-byte boundaries
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
