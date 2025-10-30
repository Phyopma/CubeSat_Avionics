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

// CubeMX instances expected
extern SPI_HandleTypeDef hspi3;
extern UART_HandleTypeDef huart2;

// GPIO mapping from your CubeMX
#define IMU_CS_GPIO_Port   GPIOB
#define IMU_CS_Pin         GPIO_PIN_2
#define IMU_RST_GPIO_Port  GPIOB
#define IMU_RST_Pin        GPIO_PIN_5
#define IMU_INT_GPIO_Port  GPIOA
#define IMU_INT_Pin        GPIO_PIN_10

// SHTP constants (host interface to SH-2 on BNO08x)
#define SHTP_MAX_PAYLOAD       128
#define SHTP_NUM_CHANNELS      6
#define SHTP_CHANNEL_CONTROL   0
#define SHTP_CHANNEL_EXECUTABLE 1
#define SHTP_CHANNEL_REPORTS   2
#define SHTP_CHANNEL_WAKE      3
#define SHTP_CHANNEL_GYRO      4
#define SHTP_CHANNEL_CERT      5

// SH-2 feature report IDs (subset)
#define SHTP_REPORT_PRODUCT_ID          0xF8
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD
#define SHTP_REPORT_BASE_TIMESTAMP      0xFB
#define SHTP_REPORT_ROTATION_VECTOR     0x05  // Quaternion (normalized)

typedef struct {
    float i, j, k, real;
    uint32_t timestamp_us;
    bool valid;
} bno085_quat_t;

typedef struct {
    // tx/rx sequence numbers per channel
    uint8_t tx_seq[SHTP_NUM_CHANNELS];
    uint8_t rx_seq[SHTP_NUM_CHANNELS];
    // rx buffer and tx buffer
    uint8_t rx_hdr[4];
    uint8_t rx_buf[SHTP_MAX_PAYLOAD];
    uint8_t tx_buf[SHTP_MAX_PAYLOAD + 4];
    // last quaternion
    bno085_quat_t quat;
} bno085_t;

// Public API
void BNO085_Reset(void);
bool BNO085_Begin(bno085_t* dev);
bool BNO085_EnableRotationVector(bno085_t* dev, uint16_t interval_us);
bool BNO085_Service(bno085_t* dev); // call when INT low or poll periodically
bool BNO085_GetQuaternion(bno085_t* dev, bno085_quat_t* out);

#endif /* APPLICATION_DRIVERS_INC_IMU_BNO085_H_ */
