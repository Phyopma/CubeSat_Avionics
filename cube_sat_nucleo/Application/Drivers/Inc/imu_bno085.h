/*
 * imu_bno085.h
 *
 * Created on: Oct 30, 2025
 * Author: phyopyae
 */

#ifndef APPLICATION_DRIVERS_INC_IMU_BNO085_H_
#define APPLICATION_DRIVERS_INC_IMU_BNO085_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// CubeMX instances
extern SPI_HandleTypeDef hspi3;
extern UART_HandleTypeDef huart2; // For logging

// GPIO mapping
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
#define SHTP_CHANNEL_CONTROL   2  // <--- COMMANDS GO HERE
#define SHTP_CHANNEL_REPORTS   3  // <--- DATA COMES HERE
#define SHTP_CHANNEL_WAKE_REPORTS 4
#define SHTP_CHANNEL_GYRO_ROTATION_VECTOR 5

// SH-2 Report IDs
#define SHTP_REPORT_SET_FEATURE_COMMAND  0xFD
#define SHTP_REPORT_PRODUCT_ID           0xF1
#define SHTP_REPORT_BASE_TIMESTAMP       0xFB
#define SHTP_REPORT_TIMESTAMP_REBASE     0xFA

// Sensor Report IDs
#define SHTP_REPORT_ROTATION_VECTOR      0x05  // (Channel 3)
#define SHTP_REPORT_GAME_ROTATION_VECTOR 0x08  // (Channel 3) No Mag
#define SHTP_REPORT_GYROSCOPE            0x02  // (Channel 3) Calibrated
#define SHTP_REPORT_MAGNETIC_FIELD       0x03  // (Channel 3) Calibrated
#define SHTP_REPORT_LINEAR_ACCELERATION  0x04  // (Channel 3) No Gravity
#define SHTP_REPORT_GYRO_ROTATION_VECTOR 0x20  // (Channel 5) Low-latency (Optional)


/**
 * @brief Generic 3-axis vector (x, y, z)
 */
typedef struct {
    float x;
    float y;
    float z;
    bool valid; // New data available?
} bno085_vec3_t;

/**
 * @brief Quaternion data structure (i, j, k, real)
 */
typedef struct {
    float i, j, k, real;
    float accuracy_rad; // Estimation of heading accuracy (rad)
    bool valid;
} bno085_quat_t;

/**
 * @brief Main BNO085 device structure
 */
typedef struct {
    // tx/rx sequence numbers per channel
    uint8_t tx_seq[SHTP_NUM_CHANNELS];

    // Reassembly buffer
    uint8_t rx_buf[SHTP_REASSEMBLY_BUF_SIZE];
    uint8_t tx_buf[SHTP_REASSEMBLY_BUF_SIZE];

    // --- Sensor Data Storage ---
    bno085_quat_t quat;          // Rotation Vector (Fused 9-axis)
    bno085_quat_t game_quat;     // Game Rot Vector (Fused 6-axis, no mag)
    bno085_vec3_t lin_accel;     // Linear Acceleration (m/s^2)
    bno085_vec3_t gyro;          // Calibrated Gyro (rad/s)
    bno085_vec3_t mag;           // Magnetic Field (uTesla)

} bno085_t;

// --- Core Functions ---
bool BNO085_Begin(bno085_t* dev);
void BNO085_Reset(void);
bool BNO085_Service(bno085_t* dev, uint8_t* channel_read);
void BNO085_Log(const char* fmt, ...);

// --- Enable Sensors (Commands sent to Channel 2) ---
bool BNO085_EnableRotationVector(bno085_t* dev, uint32_t interval_us);
bool BNO085_EnableGameRotationVector(bno085_t* dev, uint32_t interval_us);
bool BNO085_EnableLinearAccelerometer(bno085_t* dev, uint32_t interval_us);
bool BNO085_EnableGyroscope(bno085_t* dev, uint32_t interval_us);
bool BNO085_EnableMagnetometer(bno085_t* dev, uint32_t interval_us);

// --- Get Data ---
bool BNO085_GetQuaternion(bno085_t* dev, bno085_quat_t* out);
bool BNO085_GetGameQuaternion(bno085_t* dev, bno085_quat_t* out);
bool BNO085_GetLinearAcceleration(bno085_t* dev, bno085_vec3_t* out);
bool BNO085_GetGyroscope(bno085_t* dev, bno085_vec3_t* out);
bool BNO085_GetMagnetometer(bno085_t* dev, bno085_vec3_t* out);

#endif /* APPLICATION_DRIVERS_INC_IMU_BNO085_H_ */
