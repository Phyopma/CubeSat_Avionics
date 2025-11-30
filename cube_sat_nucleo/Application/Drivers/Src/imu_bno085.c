/*
 * imu_bno085.c
 */

#include "imu_bno085.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

// 100ms timeout for blocking SPI
#define SPI_TIMEOUT 100

// --- Scaling Factors (from SH-2 Reference Manual Section 6.5) ---
// Rotation Vector (Q14)
#define SCALE_Q14 (1.0f / 16384.0f)
// Linear Acceleration (Q8) -> m/s^2
#define SCALE_Q8  (1.0f / 256.0f)
// Gyroscope (Q9) -> rad/s
#define SCALE_Q9  (1.0f / 512.0f)
// Magnetic Field (Q4) -> uTesla
#define SCALE_Q4  (1.0f / 16.0f)

// --- Local SPI Helper Functions ---

static void cs_select(void) {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static void cs_deselect(void) {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef spi_rx(uint8_t* pData, uint16_t Size) {
    return HAL_SPI_Receive(&hspi3, pData, Size, SPI_TIMEOUT);
}

static HAL_StatusTypeDef spi_tx(uint8_t* pData, uint16_t Size) {
    return HAL_SPI_Transmit(&hspi3, pData, Size, SPI_TIMEOUT);
}

// --- SHTP Packet Functions ---

static bool read_packet(bno085_t* dev, uint8_t* channel, uint16_t* payload_len) {
    uint16_t total_payload_received = 0;
    uint8_t ch = 0;
    bool continuation;

    memset(dev->rx_buf, 0, SHTP_REASSEMBLY_BUF_SIZE);

    do {
        uint8_t hdr[4];
        cs_select();

        if (spi_rx(hdr, 4) != HAL_OK) {
            cs_deselect();
            return false;
        }

        uint16_t fragment_len = (uint16_t)(hdr[1] << 8) | hdr[0];
        continuation = (fragment_len & 0x8000) != 0;
        fragment_len &= 0x7FFF;

        if (fragment_len < 4) {
             cs_deselect();
             if (fragment_len == 0) continue;
             return false;
        }

        ch = hdr[2];
        uint16_t payload_in_fragment = fragment_len - 4;

        if (payload_in_fragment > 0) {
            if (total_payload_received + payload_in_fragment > SHTP_REASSEMBLY_BUF_SIZE) {
                // Overflow protection
                uint16_t bytes_to_discard = payload_in_fragment;
                uint8_t sink[32];
                while (bytes_to_discard > 0) {
                    uint16_t chunk = (bytes_to_discard > sizeof(sink)) ? sizeof(sink) : bytes_to_discard;
                    if (spi_rx(sink, chunk) != HAL_OK) {
                        cs_deselect();
                        return false;
                    }
                    bytes_to_discard -= chunk;
                }
            } else {
                if (spi_rx(&dev->rx_buf[total_payload_received], payload_in_fragment) != HAL_OK) {
                    cs_deselect();
                    return false;
                }
                total_payload_received += payload_in_fragment;
            }
        }
        cs_deselect();

    } while (continuation);

    *channel = ch;
    *payload_len = total_payload_received;
    return true;
}

static bool write_packet(bno085_t* dev, uint8_t channel, const uint8_t* payload, uint16_t payload_len) {
    uint16_t len = payload_len + 4;

    dev->tx_buf[0] = (uint8_t)(len & 0xFF);
    dev->tx_buf[1] = (uint8_t)((len >> 8) & 0x7F);
    dev->tx_buf[2] = channel;
    dev->tx_buf[3] = dev->tx_seq[channel]++;

    if (payload_len > 0 && payload != NULL) {
        memcpy(&dev->tx_buf[4], payload, payload_len);
    }

    cs_select();
    HAL_StatusTypeDef st = spi_tx(dev->tx_buf, len);
    cs_deselect();

    return (st == HAL_OK);
}

/**
 * @brief Generic Helper to send "Set Feature" commands
 * @note  ALWAYS writes to SHTP_CHANNEL_CONTROL (2)
 */
static bool BNO085_SetFeature(bno085_t* dev, uint8_t report_id, uint32_t interval_us) {
    uint8_t payload[17];
    payload[0] = SHTP_REPORT_SET_FEATURE_COMMAND; // 0xFD
    payload[1] = report_id;                       // e.g., 0x05, 0x08, 0x02...
    payload[2] = 0x00; // Feature flags
    payload[3] = 0x00; // Change Sensitivity (LSB)
    payload[4] = 0x00; // Change Sensitivity (MSB)
    payload[5] = (uint8_t)(interval_us & 0xFF);
    payload[6] = (uint8_t)((interval_us >> 8) & 0xFF);
    payload[7] = (uint8_t)((interval_us >> 16) & 0xFF);
    payload[8] = (uint8_t)((interval_us >> 24) & 0xFF);
    memset(&payload[9], 0, 8);

    // Enforce SHTP_CHANNEL_CONTROL (Channel 2)
    return write_packet(dev, SHTP_CHANNEL_CONTROL, payload, sizeof(payload));
}

// --- Report Parsing ---

static void parse_reports(bno085_t* dev, const uint8_t* p, uint16_t n) {
    uint16_t idx = 0;
    while (idx < n) {
        uint8_t report_id = p[idx];
        uint16_t report_len = 0;

        // Determine report length based on ID
        switch (report_id) {
            case SHTP_REPORT_ROTATION_VECTOR:
                report_len = 14; // ID(1)+Seq(1)+Stat(1)+Delay(1) + Data(10)
                break;
            case SHTP_REPORT_GAME_ROTATION_VECTOR:
                report_len = 12; // ID(1)+Seq(1)+Stat(1)+Delay(1) + Data(8)
                break;
            case SHTP_REPORT_LINEAR_ACCELERATION:
            case SHTP_REPORT_GYROSCOPE:
            case SHTP_REPORT_MAGNETIC_FIELD:
                report_len = 10; // ID(1)+Seq(1)+Stat(1)+Delay(1) + Data(6)
                break;

            // *** THE MISSING FIX IS HERE ***
            case SHTP_REPORT_TIMESTAMP_REBASE:
                report_len = 5; // ID(1)+Seq(1)+Stat(1)+Delay(1) + Time(1)
                break;

            case SHTP_REPORT_BASE_TIMESTAMP:
                report_len = 5;
                break;
            case SHTP_REPORT_SET_FEATURE_COMMAND:
                report_len = 17;
                break;
            case SHTP_REPORT_PRODUCT_ID:
                report_len = 16;
                break;
            default:
                report_len = 0; // Still unknown
                break;
        }

        // Safety check: Ensure we don't read past the buffer
        if (report_len > 0 && (idx + report_len <= n)) {

            // If it's a data report, parse the payload
            // (Skip ID, Seq, Status, Delay -> start at idx+4)
            uint16_t data_idx = idx + 4;

            if (report_id == SHTP_REPORT_ROTATION_VECTOR) {
                int16_t qi = (int16_t)((p[data_idx+1]<<8) | p[data_idx]);
                int16_t qj = (int16_t)((p[data_idx+3]<<8) | p[data_idx+2]);
                int16_t qk = (int16_t)((p[data_idx+5]<<8) | p[data_idx+4]);
                int16_t qr = (int16_t)((p[data_idx+7]<<8) | p[data_idx+6]);
                int16_t acc= (int16_t)((p[data_idx+9]<<8) | p[data_idx+8]);

                dev->quat.i = (float)qi * SCALE_Q14;
                dev->quat.j = (float)qj * SCALE_Q14;
                dev->quat.k = (float)qk * SCALE_Q14;
                dev->quat.real = (float)qr * SCALE_Q14;
                dev->quat.accuracy_rad = (float)acc * SCALE_Q14;
                dev->quat.valid = true;
            }
            else if (report_id == SHTP_REPORT_GAME_ROTATION_VECTOR) {
                int16_t qi = (int16_t)((p[data_idx+1]<<8) | p[data_idx]);
                int16_t qj = (int16_t)((p[data_idx+3]<<8) | p[data_idx+2]);
                int16_t qk = (int16_t)((p[data_idx+5]<<8) | p[data_idx+4]);
                int16_t qr = (int16_t)((p[data_idx+7]<<8) | p[data_idx+6]);

                dev->game_quat.i = (float)qi * SCALE_Q14;
                dev->game_quat.j = (float)qj * SCALE_Q14;
                dev->game_quat.k = (float)qk * SCALE_Q14;
                dev->game_quat.real = (float)qr * SCALE_Q14;
                dev->game_quat.valid = true;
            }
            else if (report_id == SHTP_REPORT_LINEAR_ACCELERATION) {
                int16_t x = (int16_t)((p[data_idx+1]<<8) | p[data_idx]);
                int16_t y = (int16_t)((p[data_idx+3]<<8) | p[data_idx+2]);
                int16_t z = (int16_t)((p[data_idx+5]<<8) | p[data_idx+4]);

                dev->lin_accel.x = (float)x * SCALE_Q8;
                dev->lin_accel.y = (float)y * SCALE_Q8;
                dev->lin_accel.z = (float)z * SCALE_Q8;
                dev->lin_accel.valid = true;
            }
            else if (report_id == SHTP_REPORT_GYROSCOPE) {
                int16_t x = (int16_t)((p[data_idx+1]<<8) | p[data_idx]);
                int16_t y = (int16_t)((p[data_idx+3]<<8) | p[data_idx+2]);
                int16_t z = (int16_t)((p[data_idx+5]<<8) | p[data_idx+4]);

                dev->gyro.x = (float)x * SCALE_Q9;
                dev->gyro.y = (float)y * SCALE_Q9;
                dev->gyro.z = (float)z * SCALE_Q9;
                dev->gyro.valid = true;
            }
            else if (report_id == SHTP_REPORT_MAGNETIC_FIELD) {
                int16_t x = (int16_t)((p[data_idx+1]<<8) | p[data_idx]);
                int16_t y = (int16_t)((p[data_idx+3]<<8) | p[data_idx+2]);
                int16_t z = (int16_t)((p[data_idx+5]<<8) | p[data_idx+4]);

                dev->mag.x = (float)x * SCALE_Q4;
                dev->mag.y = (float)y * SCALE_Q4;
                dev->mag.z = (float)z * SCALE_Q4;
                dev->mag.valid = true;
            }

            // Move to the next report in the buffer
            idx += report_len;
        }
        else {
            // Unknown ID or truncated buffer.
            // We MUST break to avoid infinite loops or reading garbage.
            break;
        }
    }
}

// --- Public API Functions ---

void BNO085_Reset(void) {
    BNO085_Log("BNO085_Reset: Toggling RST pin...\r\n");
    cs_deselect();
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(15);
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
}

bool BNO085_WaitForAck(bno085_t* dev) {
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 500) {
        uint8_t ch;
        if (BNO085_Service(dev, &ch)) {
            if (ch == SHTP_CHANNEL_CONTROL) {
                return true;
            }
        }
    }
    return false;
}

bool BNO085_Begin(bno085_t* dev) {
    memset(dev, 0, sizeof(*dev));
    BNO085_Reset();

    BNO085_Log("Waiting for BNO085 to boot...\r\n");
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 2000) {
        uint8_t ch;
        BNO085_Service(dev, &ch); // Flush advertisement packets
        HAL_Delay(1);
    }

    // Enable Sensors
    BNO085_Log("Enabling Rotation Vector...\r\n");
    if (!BNO085_EnableRotationVector(dev, 10000)) return false;
    if (!BNO085_WaitForAck(dev)) return false;

    return true;
}

// Wrappers for Enabling Sensors
bool BNO085_EnableRotationVector(bno085_t* dev, uint32_t interval_us) {
    return BNO085_SetFeature(dev, SHTP_REPORT_ROTATION_VECTOR, interval_us);
}
bool BNO085_EnableGameRotationVector(bno085_t* dev, uint32_t interval_us) {
    return BNO085_SetFeature(dev, SHTP_REPORT_GAME_ROTATION_VECTOR, interval_us);
}
bool BNO085_EnableLinearAccelerometer(bno085_t* dev, uint32_t interval_us) {
    return BNO085_SetFeature(dev, SHTP_REPORT_LINEAR_ACCELERATION, interval_us);
}
bool BNO085_EnableGyroscope(bno085_t* dev, uint32_t interval_us) {
    return BNO085_SetFeature(dev, SHTP_REPORT_GYROSCOPE, interval_us);
}
bool BNO085_EnableMagnetometer(bno085_t* dev, uint32_t interval_us) {
    return BNO085_SetFeature(dev, SHTP_REPORT_MAGNETIC_FIELD, interval_us);
}

bool BNO085_Service(bno085_t* dev, uint8_t* channel_read) {
    if (HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin) != GPIO_PIN_RESET) {
        return false;
    }

    uint8_t ch; uint16_t n;
    if (!read_packet(dev, &ch, &n)) {
        return false;
    }

    if (channel_read != NULL) *channel_read = ch;

//    if (ch == SHTP_CHANNEL_REPORTS || ch == SHTP_CHANNEL_GYRO_ROTATION_VECTOR) {
    if (ch == SHTP_CHANNEL_REPORTS){
        parse_reports(dev, dev->rx_buf, n);
    }

    return true;
}

// --- Getters ---

bool BNO085_GetQuaternion(bno085_t* dev, bno085_quat_t* out) {
    if (!dev->quat.valid) return false;
    *out = dev->quat;
    dev->quat.valid = false;
    return true;
}

bool BNO085_GetGameQuaternion(bno085_t* dev, bno085_quat_t* out) {
    if (!dev->game_quat.valid) return false;
    *out = dev->game_quat;
    dev->game_quat.valid = false;
    return true;
}

bool BNO085_GetLinearAcceleration(bno085_t* dev, bno085_vec3_t* out) {
    if (!dev->lin_accel.valid) return false;
    *out = dev->lin_accel;
    dev->lin_accel.valid = false;
    return true;
}

bool BNO085_GetGyroscope(bno085_t* dev, bno085_vec3_t* out) {
    if (!dev->gyro.valid) return false;
    *out = dev->gyro;
    dev->gyro.valid = false;
    return true;
}

bool BNO085_GetMagnetometer(bno085_t* dev, bno085_vec3_t* out) {
    if (!dev->mag.valid) return false;
    *out = dev->mag;
    dev->mag.valid = false;
    return true;
}

void BNO085_Log(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
}
