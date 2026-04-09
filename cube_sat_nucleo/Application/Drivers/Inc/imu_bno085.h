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
#define SHTP_REPORT_ACCELEROMETER        0x01  // (Channel 3) Calibrated
#define SHTP_REPORT_ROTATION_VECTOR      0x05  // (Channel 3)
#define SHTP_REPORT_GAME_ROTATION_VECTOR 0x08  // (Channel 3) No Mag
#define SHTP_REPORT_GYROSCOPE            0x02  // (Channel 3) Calibrated
#define SHTP_REPORT_MAGNETIC_FIELD       0x03  // (Channel 3) Calibrated
#define SHTP_REPORT_LINEAR_ACCELERATION  0x04  // (Channel 3) No Gravity
#define SHTP_REPORT_GRAVITY              0x06  // (Channel 3)
#define SHTP_REPORT_GYROSCOPE_UNCAL      0x07  // (Channel 3)
#define SHTP_REPORT_GEOMAGNETIC_RV       0x09  // (Channel 3)
#define SHTP_REPORT_MAGNETIC_FIELD_UNCAL 0x0F  // (Channel 3)
#define SHTP_REPORT_RAW_ACCELEROMETER    0x14  // (Channel 3)
#define SHTP_REPORT_RAW_GYROSCOPE        0x15  // (Channel 3)
#define SHTP_REPORT_RAW_MAGNETOMETER     0x16  // (Channel 3)
#define SHTP_REPORT_ARVR_STAB_RV         0x28  // (Channel 3)
#define SHTP_REPORT_ARVR_STAB_GRV        0x29  // (Channel 3)
#define SHTP_REPORT_GYRO_ROTATION_VECTOR 0x20  // (Channel 5) Low-latency (Optional)


typedef struct {
    float x;
    float y;
    float z;
    uint8_t status;
    uint8_t accuracy;
    uint32_t local_timestamp_ms;
    uint32_t sensor_timestamp_us;
    uint32_t sample_sequence;
    bool valid;
} bno085_vec3_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t status;
    uint8_t accuracy;
    uint32_t local_timestamp_ms;
    uint32_t sensor_timestamp_us;
    uint32_t sample_sequence;
    bool valid;
} bno085_raw_vec3_t;

typedef struct {
    float i, j, k, real;
    float accuracy_rad;
    uint8_t status;
    uint8_t accuracy;
    uint32_t local_timestamp_ms;
    uint32_t sensor_timestamp_us;
    uint32_t sample_sequence;
    bool valid;
} bno085_quat_t;

typedef struct {
    bool enabled;
    bool acknowledged;
    uint32_t interval_us;
    uint32_t last_update_ms;
} bno085_feature_state_t;

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
    bno085_vec3_t accel;         // Calibrated Accelerometer (m/s^2)
    bno085_vec3_t gravity;       // Gravity vector (m/s^2)
    bno085_vec3_t gyro;          // Calibrated Gyro (rad/s)
    bno085_vec3_t mag_cal;       // Calibrated magnetic field (uTesla)
    bno085_vec3_t mag_uncal;     // Uncalibrated magnetic field (uTesla)
    bno085_raw_vec3_t mag_raw;   // Raw magnetic field (ADC counts)
    bno085_feature_state_t feature_state[256];
    uint32_t last_base_timestamp_us;
    uint32_t last_local_timestamp_ms;

} bno085_t;

// --- Core Functions ---
bool BNO085_Begin(bno085_t* dev);
void BNO085_Reset(void);
bool BNO085_Service(bno085_t* dev, uint8_t* channel_read);
void BNO085_Log(const char* fmt, ...);
bool BNO085_WaitForAck(bno085_t* dev);
bool BNO085_ConfigureFeature(bno085_t* dev, uint8_t report_id, uint32_t interval_us);
bool BNO085_RequestFeatureState(bno085_t* dev, uint8_t report_id);
bool BNO085_GetFeatureState(const bno085_t* dev, uint8_t report_id, bno085_feature_state_t* out);

// --- Enable Sensors (Commands sent to Channel 2) ---
bool BNO085_EnableRotationVector(bno085_t* dev, uint32_t interval_us);
bool BNO085_EnableGameRotationVector(bno085_t* dev, uint32_t interval_us);
bool BNO085_EnableLinearAccelerometer(bno085_t* dev, uint32_t interval_us);
bool BNO085_EnableAccelerometer(bno085_t* dev, uint32_t interval_us);
bool BNO085_EnableGravity(bno085_t* dev, uint32_t interval_us);
bool BNO085_EnableGyroscope(bno085_t* dev, uint32_t interval_us);
bool BNO085_EnableMagnetometer(bno085_t* dev, uint32_t interval_us);
bool BNO085_EnableMagnetometerUncal(bno085_t* dev, uint32_t interval_us);
bool BNO085_EnableRawMagnetometer(bno085_t* dev, uint32_t interval_us);

// --- Get Data ---
bool BNO085_CopyQuaternion(const bno085_t* dev, bno085_quat_t* out);
bool BNO085_CopyGameQuaternion(const bno085_t* dev, bno085_quat_t* out);
bool BNO085_CopyLinearAcceleration(const bno085_t* dev, bno085_vec3_t* out);
bool BNO085_CopyAccelerometer(const bno085_t* dev, bno085_vec3_t* out);
bool BNO085_CopyGravity(const bno085_t* dev, bno085_vec3_t* out);
bool BNO085_CopyGyroscope(const bno085_t* dev, bno085_vec3_t* out);
bool BNO085_CopyMagnetometerCal(const bno085_t* dev, bno085_vec3_t* out);
bool BNO085_CopyMagnetometerUncal(const bno085_t* dev, bno085_vec3_t* out);
bool BNO085_CopyRawMagnetometer(const bno085_t* dev, bno085_raw_vec3_t* out);

#endif /* APPLICATION_DRIVERS_INC_IMU_BNO085_H_ */
