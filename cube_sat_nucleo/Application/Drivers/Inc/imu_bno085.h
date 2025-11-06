/*
 * imu_bno085.h
 *
 *  Created on: Oct 30, 2025
 *      Author: phyopyae
 */

#ifndef APPLICATION_DRIVERS_INC_IMU_BNO085_H_
#define APPLICATION_DRIVERS_INC_IMU_BNO085_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>


// CubeMX instances
extern SPI_HandleTypeDef hspi3;
extern UART_HandleTypeDef huart2; // For logging

// GPIO mapping from CubeMX
#define IMU_CS_GPIO_Port   GPIOB
#define IMU_CS_Pin         GPIO_PIN_2
#define IMU_RST_GPIO_Port  GPIOB
#define IMU_RST_Pin        GPIO_PIN_5
#define IMU_INT_GPIO_Port  GPIOA
#define IMU_INT_Pin        GPIO_PIN_10

// SHTP constants
#define SHTP_REASSEMBLY_BUF_SIZE 1024
#define SHTP_NUM_CHANNELS      6
#define SHTP_CHANNEL_COMMAND   0
#define SHTP_CHANNEL_EXECUTABLE 1
#define SHTP_CHANNEL_CONTROL   2
#define SHTP_CHANNEL_REPORTS   3
#define SHTP_CHANNEL_WAKE_REPORTS 4
#define SHTP_CHANNEL_GYRO_ROTATION_VECTOR 5

// SH-2 report IDs
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD
#define SHTP_REPORT_BASE_TIMESTAMP      0xFB
#define SHTP_REPORT_ROTATION_VECTOR     0x05
#define SHTP_REPORT_PRODUCT_ID          0xF1

/**
 * @brief Quaternion data structure
 */
typedef struct {
    float i, j, k, real;
    uint32_t timestamp_us;
    bool valid;
} bno085_quat_t;

/**
 * @brief Main BNO085 device structure
 */
typedef struct {
    // tx/rx sequence numbers per channel
    uint8_t tx_seq[SHTP_NUM_CHANNELS];

    // Reassembly buffer for fragmented SHTP packets
    uint8_t rx_buf[SHTP_REASSEMBLY_BUF_SIZE];
    uint8_t tx_buf[SHTP_REASSEMBLY_BUF_SIZE];

    // Last received quaternion
    bno085_quat_t quat;
} bno085_t;


/**
 * @brief Performs a hardware reset and initializes the BNO085.
 * @param dev Pointer to the device structure.
 * @return true on success, false on failure.
 */
bool BNO085_Begin(bno085_t* dev);

/**
 * @brief Must be called in the main loop to process incoming data.
 * @param dev Pointer to the device structure.
 * @return true if a report was processed, false otherwise.
 */
bool BNO085_Service(bno085_t* dev, uint8_t* channel_read); // change docs later

/**
 * @brief Gets the latest valid quaternion reading.
 * @param dev Pointer to the device structure.
 * @param out Pointer to store the output quaternion.
 * @return true if a new, valid reading was returned.
 */
bool BNO085_GetQuaternion(bno085_t* dev, bno085_quat_t* out);

/**
 * @brief Performs a hardware reset of the BNO085.
 */
void BNO085_Reset(void);

/**
 * @brief Enables the rotation vector report at a given interval.
 * @param dev Pointer to the device structure.
 * @param interval_us Report interval in microseconds.
 * @return true on successful command send, false on SPI fail.
 */
bool BNO085_EnableRotationVector(bno085_t* dev, uint32_t interval_us);

/**
 * @brief Helper function for printf-style logging over UART
 */
void BNO085_Log(const char* fmt, ...);

#endif /* APPLICATION_DRIVERS_INC_IMU_BNO085_H_ */
