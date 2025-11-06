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
                // Payload overflow, we must discard the rest of this fragment
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
                // Read the payload that fits
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

// --- Report Parsing ---

static void parse_reports(bno085_t* dev, const uint8_t* p, uint16_t n) {
    uint16_t idx = 0;
    while (idx < n) {
        uint8_t report_id = p[idx];

        uint16_t report_len = 0;
        if (report_id == SHTP_REPORT_ROTATION_VECTOR) {
            report_len = 14;
        } else if (report_id == SHTP_REPORT_SET_FEATURE_COMMAND) {
            report_len = 17;
        } else if (report_id == SHTP_REPORT_BASE_TIMESTAMP) {
            report_len = 5;
        } else if (report_id == SHTP_REPORT_PRODUCT_ID) {
            report_len = 16;
        }

        if (report_len > 0 && (idx + report_len <= n)) {
            if (report_id == SHTP_REPORT_ROTATION_VECTOR) {
                int16_t qi = (int16_t)((p[idx+5]<<8) | p[idx+4]);
                int16_t qj = (int16_t)((p[idx+7]<<8) | p[idx+6]);
                int16_t qk = (int16_t)((p[idx+9]<<8) | p[idx+8]);
                int16_t qr = (int16_t)((p[idx+11]<<8) | p[idx+10]);

                const float scale = 1.0f / 16384.0f; // Q14 format
                dev->quat.i = (float)qi * scale;
                dev->quat.j = (float)qj * scale;
                dev->quat.k = (float)qk * scale;
                dev->quat.real = (float)qr * scale;
                dev->quat.timestamp_us = 0;
                dev->quat.valid = true;
            }
            idx += report_len;

        } else if (report_len == 0) {
            // Ignoring unknown report ID
            break;

        } else {
            // Truncated report
            break;
        }
    }
}

// --- Public API Functions ---

void BNO085_Reset(void) {
    BNO085_Log("BNO085_Reset: Toggling RST pin...\n");
    cs_deselect();
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(15);
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
}

/**
 * @brief Helper function to wait for a command response
 * @return true on ACK, false on timeout
 */
static bool BNO085_WaitForAck(bno085_t* dev) {
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 500) { // Increased timeout for ACK
        uint8_t ch;
        if (BNO085_Service(dev, &ch)) {
            if (ch == SHTP_CHANNEL_CONTROL) {
                BNO085_Log("...Got ACK on channel 2.\n");
                return true; // Received the "Command Response"
            }
        }
    }
    BNO085_Log("...ACK timeout.\n");
    return false;
}

bool BNO085_Begin(bno085_t* dev) {
    memset(dev, 0, sizeof(*dev));
    BNO085_Reset();

    BNO085_Log("Waiting for BNO085 to boot (2000ms)...\n");

    // Wait 2.0 seconds for the BNO085 to boot and send all its
    // unsolicited advertisement packets.
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 2000) {
        uint8_t ch;
        (void)BNO085_Service(dev, &ch); // Call to read/discard
        HAL_Delay(1);
    }

    // Send "Enable Rotation Vector"
    BNO085_Log("Sending Enable Rotation Vector command...\n");
    if (!BNO085_EnableRotationVector(dev, 10000)) {
        BNO085_Log("Enable Rotation Vector write failed\n");
        return false;
    }
    // Wait for ACK
    if (!BNO085_WaitForAck(dev)) {
        BNO085_Log("No ACK for Rotation Vector.\n");
        return false;
    }

    // Wait up to 500 ms for the first valid data report
    BNO085_Log("Waiting for first quaternion...\n");
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 500) {
        uint8_t ch;
        if (BNO085_Service(dev, &ch)) {
            if (dev->quat.valid) {
                BNO085_Log("Got first valid quaternion!\n");
                return true;
            }
        }
    }

    BNO085_Log("Init timeout, no valid quat received.\n");
    return false;
}

bool BNO085_EnableRotationVector(bno085_t* dev, uint32_t interval_us) {
    uint8_t payload[17];
    payload[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    payload[1] = SHTP_REPORT_ROTATION_VECTOR;
    payload[2] = 0x00; // Feature flags
    payload[3] = 0x00; // Change Sensitivity (LSB)
    payload[4] = 0x00; // Change Sensitivity (MSB)
    payload[5] = (uint8_t)(interval_us & 0xFF);
    payload[6] = (uint8_t)((interval_us >> 8) & 0xFF);
    payload[7] = (uint8_t)((interval_us >> 16) & 0xFF);
    payload[8] = (uint8_t)((interval_us >> 24) & 0xFF);
    memset(&payload[9], 0, 8);

    // *** THIS IS YOUR CONFIRMED WORKING CHANNEL ***
    // Sending on Channel 2 (SHTP_CHANNEL_CONTROL)
    return write_packet(dev, SHTP_CHANNEL_CONTROL, payload, sizeof(payload));
}

bool BNO085_Service(bno085_t* dev, uint8_t* channel_read) {
    if (HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin) != GPIO_PIN_RESET) {
        return false;
    }

    // INT pin is LOW, so we have data.
    uint8_t ch; uint16_t n;
    if (!read_packet(dev, &ch, &n)) {
        return false;
    }

    if (channel_read != NULL) {
        *channel_read = ch;
    }

    // Parse reports from main reports channel (3) or gyro_rv channel (5)
    if (ch == SHTP_CHANNEL_REPORTS || ch == SHTP_CHANNEL_GYRO_ROTATION_VECTOR) {
        parse_reports(dev, dev->rx_buf, n);
    }

    return true;
}

bool BNO085_GetQuaternion(bno085_t* dev, bno085_quat_t* out) {
    if (!dev->quat.valid) return false;
    *out = dev->quat;
    dev->quat.valid = false;
    return true;
}

void BNO085_Log(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
}
