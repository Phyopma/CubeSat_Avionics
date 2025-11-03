/*
 * imu_bno085.c
 *
 *  Created on: Oct 30, 2025
 *      Author: phyopyae
 */

#include "imu_bno085.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

// 100ms timeout for blocking SPI
#define SPI_TIMEOUT 100

static void cs_select(void) {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief De-assert CS pin (Inactive HIGH)
 */
static void cs_deselect(void) {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief [NEW] Full-duplex SPI transmit/receive helper
 */
static HAL_StatusTypeDef spi_txrx(uint8_t* tx_buf, uint8_t* rx_buf, uint16_t len) {
    return HAL_SPI_TransmitReceive(&hspi3, tx_buf, rx_buf, len, SPI_TIMEOUT);
}

/**
 * @brief Simple blocking SPI transmit (Write-only)
 */
static HAL_StatusTypeDef spi_tx(uint8_t* pData, uint16_t Size) {
    return HAL_SPI_Transmit(&hspi3, pData, Size, SPI_TIMEOUT);
}

// --- SHTP Packet Functions ---

/**
 * @brief [FIXED] Reads a complete SHTP packet, handling fragmentation
 * and discarding overflow. Now correctly uses dummy TX for SPI reads.
 * @return true on success, false on SPI error or invalid packet.
 */
//static bool read_packet(bno085_t* dev, uint8_t* channel, uint16_t* payload_len) {
//    uint16_t total_payload_received = 0;
//    uint8_t ch = 0;
//    bool continuation;
//
//    // dev->dummy_tx_buf should be pre-filled with 0xFF
//
//    do {
//        uint8_t hdr[4];
//        cs_select();
//
//        // 1. Read header by sending 4 dummy bytes (0xFF)
//        if (spi_txrx(dev->dummy_tx_buf, hdr, 4) != HAL_OK) {
//            cs_deselect();
//            return false; // SPI read error
//        }
//
//        // 2. Parse header
//        uint16_t fragment_len = (uint16_t)(hdr[1] << 8) | hdr[0];
//        continuation = (fragment_len & 0x8000) != 0;
//        fragment_len &= 0x7FFF; // Total length of this fragment (header + payload)
//
//        // 3. Validate fragment length
//        if (fragment_len == 0) {
//             cs_deselect();
//             // This is a "no-data" packet, not an error.
//             continue;
//        }
//
//        if (fragment_len < 4) {
//             cs_deselect();
//             // This could be a valid case on boot, just ignore it.
//             //BNO085_Log("Fragment length invalid: %d\n", fragment_len);
//             return false; // Error: fragment length is invalid
//        }
//
//        ch = hdr[2]; // Get channel
//        uint16_t payload_in_fragment = fragment_len - 4;
//
//        if (payload_in_fragment > 0) {
//            // 4. Calculate how much payload we can store
//            uint16_t room_in_buffer = SHTP_REASSEMBLY_BUF_SIZE - total_payload_received;
//            uint16_t bytes_to_store = (payload_in_fragment <= room_in_buffer) ? payload_in_fragment : room_in_buffer;
//            uint16_t bytes_to_discard = payload_in_fragment - bytes_to_store;
//
//            // 5. Read and store the payload that fits
//            if (bytes_to_store > 0) {
//                // Read by sending dummy bytes
//                if (spi_txrx(dev->dummy_tx_buf, &dev->rx_buf[total_payload_received], bytes_to_store) != HAL_OK) {
//                    cs_deselect();
//                    BNO085_Log("Payload SPI read error\n");
//                    return false; // SPI read error
//                }
//                total_payload_received += bytes_to_store;
//            }
//
//            // 6. Read and discard the rest of the payload that doesn't fit
//            if (bytes_to_discard > 0) {
//                BNO085_Log("Payload overflow! Discarding %d bytes.\n", (int)bytes_to_discard);
//                // We need a small sink buffer, as we can't write to NULL
//                uint8_t sink[32];
//                uint16_t left = bytes_to_discard;
//                while (left > 0) {
//                    uint16_t chunk = (left > sizeof(sink)) ? sizeof(sink) : left;
//                    // Read by sending dummy bytes, receive into sink
//                    if (spi_txrx(dev->dummy_tx_buf, sink, chunk) != HAL_OK) {
//                        cs_deselect();
//                        BNO085_Log("Payload discard SPI error\n");
//                        return false; // SPI read error
//                    }
//                    left -= chunk;
//                }
//            }
//        }
//
//        // 7. End this SPI transaction
//        cs_deselect();
//
//    } while (continuation);
//
//    *channel = ch;
//    *payload_len = total_payload_received;
//    return true;
//}



/**
 * @brief Writes a complete SHTP packet.
 * @return true on success, false on SPI error.
 */
static bool write_packet(bno085_t* dev, uint8_t channel, const uint8_t* payload, uint16_t payload_len) {
    uint16_t len = payload_len + 4;

    // Build header
    dev->tx_buf[0] = (uint8_t)(len & 0xFF);
    dev->tx_buf[1] = (uint8_t)((len >> 8) & 0x7F); // No continuation
    dev->tx_buf[2] = channel;
    dev->tx_buf[3] = dev->tx_seq[channel]++;

    // Copy payload
    if (payload_len > 0 && payload != NULL) {
        memcpy(&dev->tx_buf[4], payload, payload_len);
    }

    // Transmit header + payload
    cs_select();
    HAL_StatusTypeDef st = spi_tx(dev->tx_buf, len);
    cs_deselect();

    return (st == HAL_OK);
}

// --- Report Parsing ---

/**
 * @brief Parses incoming data from the SHTP_CHANNEL_REPORTS.
 */
static void parse_reports(bno085_t* dev, const uint8_t* p, uint16_t n) {
    uint16_t idx = 0;
    while (idx < n) {
        uint8_t report_id = p[idx];

        // Known report lengths
        uint16_t report_len = 0;
        if (report_id == SHTP_REPORT_ROTATION_VECTOR) {
            report_len = 14;
        } else if (report_id == SHTP_REPORT_SET_FEATURE_COMMAND) {
            report_len = 17;
        } else if (report_id == SHTP_REPORT_BASE_TIMESTAMP) {
            report_len = 5;
        } else if (report_id == 0xF1) { // Product ID Response
            report_len = 16;
        }

        // Check if we have the full report in the buffer
        if (report_len > 0 && (idx + report_len <= n)) {
            // We know the report and its length. Process it if we care.
            if (report_id == SHTP_REPORT_ROTATION_VECTOR) {
                // Layout: [ID(1)][Seq(1)][Status(1)][unused(1)] [i(2)][j(2)][k(2)][real(2)][accuracy(2)]
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
            // Move to the next report
            idx += report_len;

        } else if (report_len == 0) {
            // Unknown report ID. We can't parse this packet anymore.
            BNO085_Log("Unknown report ID: 0x%02X\n", report_id);
            break; // Exit the while loop

        } else {
            // Known report, but buffer is truncated.
            BNO085_Log("Truncated report: 0x%02X\n", report_id);
            break; // Exit the while loop
        }
    }
}

// --- Public API Functions ---

void BNO085_Reset(void) {
    cs_deselect();
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(15);
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50); // Allow time for boot
}

bool BNO085_Begin(bno085_t* dev) {
    memset(dev, 0, sizeof(*dev));

    // *** ADD THIS LINE ***
    // Fill dummy buffer with 0xFF as required by BNO085 SPI protocol
    memset(dev->dummy_tx_buf, 0xFF, SHTP_REASSEMBLY_BUF_SIZE);

    BNO085_Reset();

    // After reset, the BNO085 sends an "Unsolicited" init packet (0xF1).
    // We must read it to clear the INT line.
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 250) { // Wait max 250ms for init packets
        if (BNO085_Service(dev)) {
             //BNO085_Log("Got init packet\n");
        }
        HAL_Delay(1); // Small delay
    }

    // At this point, the INT line should be high and all boot packets cleared.

    // Enable Rotation Vector at 100 Hz (10,000 us)
    if (!BNO085_EnableRotationVector(dev, 10000)) {
        BNO085_Log("Enable Rotation Vector failed\n");
        return false;
    }

    // Wait up to 500 ms for first valid report
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 500) {
        if (BNO085_Service(dev)) {
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
    payload[3] = 0x00; // Sequence
    payload[4] = 0x00;
    // 32-bit interval, little endian
    payload[5] = (uint8_t)(interval_us & 0xFF);
    payload[6] = (uint8_t)((interval_us >> 8) & 0xFF);
    payload[7] = (uint8_t)((interval_us >> 16) & 0xFF);
    payload[8] = (uint8_t)((interval_us >> 24) & 0xFF);
    // 32-bit specific config (0)
    memset(&payload[9], 0, 8);

    return write_packet(dev, SHTP_CHANNEL_CONTROL, payload, sizeof(payload));
}

bool BNO085_Service(bno085_t* dev) {
    // If INT not asserted, nothing to read
    if (HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin) != GPIO_PIN_RESET) {
        return false;
    }

    uint8_t ch; uint16_t n;
    if (!read_packet(dev, &ch, &n)) {
        return false;
    }

    if (ch == SHTP_CHANNEL_REPORTS) {
        parse_reports(dev, dev->rx_buf, n);
        return true;
    }

    // You can add handlers for other channels here if needed
    return false;
}

bool BNO085_GetQuaternion(bno085_t* dev, bno085_quat_t* out) {
    if (!dev->quat.valid) return false;
    *out = dev->quat;
    dev->quat.valid = false; // Consume the reading
    return true;
}

/**
 * @brief Simple vsprintf-based logger over UART2
 */
void BNO085_Log(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
}
