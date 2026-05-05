/*
 * imu_bno085.c
 */

#include "imu_bno085.h"
#include "config.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "serial_log_dma.h"

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
#define GRAVITY_MPS2 9.80665f

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
static bool bno085_try_derive_lin_from_quat(const bno085_t *dev, bno085_vec3_t *out_lin)
{
    const bno085_quat_t *q = &dev->quat;
    float w = q->real;
    float x = q->i;
    float y = q->j;
    float z = q->k;
    float norm = (w * w) + (x * x) + (y * y) + (z * z);
    if (norm < 0.10f) {
        return false;
    }

    // Gravity direction in body frame from quaternion (Android/SH-2 convention).
    float gx = 2.0f * ((x * z) - (w * y));
    float gy = 2.0f * ((w * x) + (y * z));
    float gz = (w * w) - (x * x) - (y * y) + (z * z);

    out_lin->x = dev->accel.x - (gx * GRAVITY_MPS2);
    out_lin->y = dev->accel.y - (gy * GRAVITY_MPS2);
    out_lin->z = dev->accel.z - (gz * GRAVITY_MPS2);
    out_lin->valid = true;
    return true;
}

static uint16_t bno085_report_len_for_id(uint8_t report_id)
{
    switch (report_id) {
        case SHTP_REPORT_BASE_TIMESTAMP:
        case SHTP_REPORT_TIMESTAMP_REBASE:
            return 5U;   // ID + 4-byte delta

        case SHTP_REPORT_ACCELEROMETER:
        case SHTP_REPORT_GYROSCOPE:
        case SHTP_REPORT_MAGNETIC_FIELD:
        case SHTP_REPORT_MAGNETIC_FIELD_UNCAL:
        case SHTP_REPORT_LINEAR_ACCELERATION:
        case SHTP_REPORT_GRAVITY:
        case SHTP_REPORT_RAW_ACCELEROMETER:
        case SHTP_REPORT_RAW_GYROSCOPE:
        case SHTP_REPORT_RAW_MAGNETOMETER:
            return 10U;  // ID + Seq + Status + Delay + XYZ

        case SHTP_REPORT_ROTATION_VECTOR:
        case SHTP_REPORT_GEOMAGNETIC_RV:
        case SHTP_REPORT_ARVR_STAB_RV:
            return 14U;  // ID + Seq + Status + Delay + quat + accuracy

        case SHTP_REPORT_GAME_ROTATION_VECTOR:
        case SHTP_REPORT_ARVR_STAB_GRV:
            return 12U;  // ID + Seq + Status + Delay + quat

        case SHTP_REPORT_GYROSCOPE_UNCAL:
            return 16U;  // ID + Seq + Status + Delay + XYZ + biasXYZ

        case SHTP_REPORT_SET_FEATURE_COMMAND:
            return 17U;
        case SHTP_REPORT_PRODUCT_ID:
            return 16U;
        default:
            return 0U;
    }
}

static void parse_reports(bno085_t* dev, const uint8_t* p, uint16_t n, uint8_t channel) {
    static uint8_t seen_report_id[256] = {0};
    uint16_t idx = 0;
    while (idx < n) {
        uint8_t report_id = p[idx];
        uint16_t report_len = bno085_report_len_for_id(report_id);

        if (seen_report_id[report_id] == 0U) {
            seen_report_id[report_id] = 1U;
            log_printf_dma("BNO085 report id seen ch=%u id=0x%02X", channel, report_id);
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
            else if (report_id == SHTP_REPORT_ACCELEROMETER) {
                int16_t x = (int16_t)((p[data_idx+1]<<8) | p[data_idx]);
                int16_t y = (int16_t)((p[data_idx+3]<<8) | p[data_idx+2]);
                int16_t z = (int16_t)((p[data_idx+5]<<8) | p[data_idx+4]);
                dev->accel.x = (float)x * SCALE_Q8;
                dev->accel.y = (float)y * SCALE_Q8;
                dev->accel.z = (float)z * SCALE_Q8;
                dev->accel.valid = true;
            }
            else if (report_id == SHTP_REPORT_GRAVITY) {
                int16_t x = (int16_t)((p[data_idx+1]<<8) | p[data_idx]);
                int16_t y = (int16_t)((p[data_idx+3]<<8) | p[data_idx+2]);
                int16_t z = (int16_t)((p[data_idx+5]<<8) | p[data_idx+4]);
                dev->gravity.x = (float)x * SCALE_Q8;
                dev->gravity.y = (float)y * SCALE_Q8;
                dev->gravity.z = (float)z * SCALE_Q8;
                dev->gravity.valid = true;
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
            else if (report_id == SHTP_REPORT_MAGNETIC_FIELD_UNCAL) {
                int16_t x = (int16_t)((p[data_idx+1]<<8) | p[data_idx]);
                int16_t y = (int16_t)((p[data_idx+3]<<8) | p[data_idx+2]);
                int16_t z = (int16_t)((p[data_idx+5]<<8) | p[data_idx+4]);
                dev->mag.x = (float)x * SCALE_Q4;
                dev->mag.y = (float)y * SCALE_Q4;
                dev->mag.z = (float)z * SCALE_Q4;
                dev->mag.valid = true;
                static uint8_t logged_uncal_mag = 0U;
                if (logged_uncal_mag == 0U) {
                    logged_uncal_mag = 1U;
                    log_printf_dma("BNO085 using uncalibrated magnetic field fallback (id=0x0F)");
                }
            }

            // Fallback path when dedicated 0x04 stream is unavailable.
            if (dev->lin_accel.valid == false && dev->accel.valid == true) {
                uint8_t used_fallback = 0U;
                if (dev->gravity.valid == true) {
                    dev->lin_accel.x = dev->accel.x - dev->gravity.x;
                    dev->lin_accel.y = dev->accel.y - dev->gravity.y;
                    dev->lin_accel.z = dev->accel.z - dev->gravity.z;
                    dev->lin_accel.valid = true;
                    used_fallback = 1U;
                    static uint8_t logged_lin_gravity_fallback = 0U;
                    if (logged_lin_gravity_fallback == 0U) {
                        logged_lin_gravity_fallback = 1U;
                        log_printf_dma("BNO085 using linear accel fallback: accel-gravity");
                    }
                } else if (bno085_try_derive_lin_from_quat(dev, &dev->lin_accel)) {
                    used_fallback = 1U;
                    static uint8_t logged_lin_quat_fallback = 0U;
                    if (logged_lin_quat_fallback == 0U) {
                        logged_lin_quat_fallback = 1U;
                        log_printf_dma("BNO085 using linear accel fallback: accel-quaternion-gravity");
                    }
                }
                (void)used_fallback;
            }

            // Move to the next report in the buffer
            idx += report_len;
        }
        else {
            // Don't byte-scan on unknown/truncated reports; it can desync all
            // following report boundaries in this packet.
            static uint8_t seen_bad_id[256] = {0};
            if (seen_bad_id[report_id] == 0U) {
                seen_bad_id[report_id] = 1U;
                log_printf_dma("WARN BNO085 unknown/truncated report ch=%u id=0x%02X n=%u idx=%u",
                               channel, report_id, (unsigned)n, (unsigned)idx);
            }
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

static bool BNO085_WaitForFeatureResponse(bno085_t* dev, uint8_t feature_id, uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        uint8_t ch = 0;
        if (BNO085_Service(dev, &ch) && ch == SHTP_CHANNEL_CONTROL) {
            // Get Feature Response: report ID 0xFC, byte1 = feature report ID
            if (dev->rx_buf[0] == 0xFC) {
                uint8_t resp_feature_id = dev->rx_buf[1];
                uint32_t resp_interval_us =
                    ((uint32_t)dev->rx_buf[8] << 24) |
                    ((uint32_t)dev->rx_buf[7] << 16) |
                    ((uint32_t)dev->rx_buf[6] << 8) |
                    (uint32_t)dev->rx_buf[5];
                BNO085_Log("BNO085 feature rsp id=0x%02X interval_us=%lu\r\n",
                           resp_feature_id, (unsigned long)resp_interval_us);
                if (resp_feature_id == feature_id) {
                    return true;
                }
            }
        }
    }
    return false;
}

static bool BNO085_EnableFeatureWithResponse(bno085_t* dev,
                                             uint8_t feature_id,
                                             uint32_t interval_us,
                                             const char* name) {
    if (!BNO085_SetFeature(dev, feature_id, interval_us)) {
        return false;
    }
    if (!BNO085_WaitForFeatureResponse(dev, feature_id, 500U)) {
        BNO085_Log("WARN no feature response for %s (id=0x%02X)\r\n", name, feature_id);
        return false;
    }
    return true;
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

    // Enable sensors and verify each one is accepted by SH-2.
    BNO085_Log("Enabling Rotation Vector...\r\n");
    if (!BNO085_EnableFeatureWithResponse(dev, SHTP_REPORT_ROTATION_VECTOR, BNO085_REPORT_INTERVAL_US, "Rotation Vector")) return false;

    BNO085_Log("Enabling Game Rotation Vector...\r\n");
    if (!BNO085_EnableFeatureWithResponse(dev, SHTP_REPORT_GAME_ROTATION_VECTOR, BNO085_REPORT_INTERVAL_US, "Game Rotation Vector")) return false;

    BNO085_Log("Enabling Gyroscope...\r\n");
    if (!BNO085_EnableFeatureWithResponse(dev, SHTP_REPORT_GYROSCOPE, BNO085_REPORT_INTERVAL_US, "Gyroscope")) return false;

    BNO085_Log("Enabling Magnetic Field...\r\n");
    if (!BNO085_EnableFeatureWithResponse(dev, SHTP_REPORT_MAGNETIC_FIELD, BNO085_REPORT_INTERVAL_US, "Magnetic Field")) return false;

    BNO085_Log("Enabling Linear Acceleration...\r\n");
    if (!BNO085_EnableFeatureWithResponse(dev, SHTP_REPORT_LINEAR_ACCELERATION, BNO085_REPORT_INTERVAL_US, "Linear Acceleration")) return false;

    // Optional fallback streams for boards/firmware variants that may not
    // emit 0x03/0x04 consistently even after accepting Set Feature.
    BNO085_Log("Enabling Accelerometer (fallback)...\r\n");
    if (!BNO085_EnableFeatureWithResponse(dev, SHTP_REPORT_ACCELEROMETER, BNO085_REPORT_INTERVAL_US, "Accelerometer")) {
        BNO085_Log("WARN fallback stream unavailable: Accelerometer\r\n");
    }

    BNO085_Log("Enabling Gravity (fallback)...\r\n");
    if (!BNO085_EnableFeatureWithResponse(dev, SHTP_REPORT_GRAVITY, BNO085_REPORT_INTERVAL_US, "Gravity")) {
        BNO085_Log("WARN fallback stream unavailable: Gravity\r\n");
    }

    BNO085_Log("Enabling Magnetic Field Uncal (fallback)...\r\n");
    if (!BNO085_EnableFeatureWithResponse(dev, SHTP_REPORT_MAGNETIC_FIELD_UNCAL, BNO085_REPORT_INTERVAL_US, "Magnetic Field Uncal")) {
        BNO085_Log("WARN fallback stream unavailable: Magnetic Field Uncal\r\n");
    }

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
bool BNO085_EnableAccelerometer(bno085_t* dev, uint32_t interval_us) {
    return BNO085_SetFeature(dev, SHTP_REPORT_ACCELEROMETER, interval_us);
}
bool BNO085_EnableGravity(bno085_t* dev, uint32_t interval_us) {
    return BNO085_SetFeature(dev, SHTP_REPORT_GRAVITY, interval_us);
}
bool BNO085_EnableGyroscope(bno085_t* dev, uint32_t interval_us) {
    return BNO085_SetFeature(dev, SHTP_REPORT_GYROSCOPE, interval_us);
}
bool BNO085_EnableMagnetometer(bno085_t* dev, uint32_t interval_us) {
    return BNO085_SetFeature(dev, SHTP_REPORT_MAGNETIC_FIELD, interval_us);
}
bool BNO085_EnableMagnetometerUncal(bno085_t* dev, uint32_t interval_us) {
    return BNO085_SetFeature(dev, SHTP_REPORT_MAGNETIC_FIELD_UNCAL, interval_us);
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

    if (ch == SHTP_CHANNEL_REPORTS ||
        ch == SHTP_CHANNEL_WAKE_REPORTS ||
        ch == SHTP_CHANNEL_GYRO_ROTATION_VECTOR) {
        parse_reports(dev, dev->rx_buf, n, ch);
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
    if (len <= 0) {
        return;
    }
    log_printf_dma("%s", buf);
}
