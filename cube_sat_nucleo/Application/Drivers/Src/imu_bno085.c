/*
 * imu_bno085.c
 *
 *  Created on: Oct 30, 2025
 *      Author: phyopyae
 */

#include "imu_bno085.h"
#include <string.h>
#include <math.h>

// Local helpers
static void cs_select(void)   { HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET); }
static void cs_deselect(void) { HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET); }

static HAL_StatusTypeDef spi_txrx(uint8_t* tx, uint8_t* rx, uint16_t len) {
    return HAL_SPI_TransmitReceive(&hspi3, tx, rx, len, 10);
}

static HAL_StatusTypeDef spi_rx(uint8_t* rx, uint16_t len) {
    return HAL_SPI_Receive(&hspi3, rx, len, 10);
}

static HAL_StatusTypeDef spi_tx(uint8_t* tx, uint16_t len) {
    return HAL_SPI_Transmit(&hspi3, tx, len, 10);
}

void BNO085_Reset(void) {
    // Ensure CS deasserted
    cs_deselect();
    // Active-low reset pulse >=10 ms
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(15);
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50); // allow boot
}

//static bool read_packet(bno085_t* dev, uint8_t* channel, uint16_t* payload_len) {
//    // SPI SHTP header is 4 bytes: [len LSB][len MSB][channel][seq]
//    uint8_t hdr[4] = {0};
//    cs_select();
//    if (spi_rx(hdr, 4) != HAL_OK) { cs_deselect(); return false; }
//    uint16_t len = hdr[0] | ((uint16_t)hdr[1] << 8);
//    bool continuation = (len & 0x8000) != 0;
//    len &= 0x7FFF;
//    if (len == 0) { cs_deselect(); return false; }
//    *channel = hdr[2];
//    uint8_t seq = hdr[3];
//    (void)seq;
//    uint16_t to_read = (len > SHTP_MAX_PAYLOAD) ? SHTP_MAX_PAYLOAD : len;
//    if (spi_rx(dev->rx_buf, to_read) != HAL_OK) { cs_deselect(); return false; }
//    cs_deselect();
//    // Discard continuation bodies for simplicity (BNO keeps small packets)
//    *payload_len = to_read;
//    return !continuation;
//}

static bool read_packet(bno085_t* dev, uint8_t* channel, uint16_t* payload_len) {
    uint16_t total = 0;
    uint8_t ch = 0;
    bool cont;
    do {
        uint8_t hdr[4];
        cs_select();
        if (spi_rx(hdr, 4) != HAL_OK) { cs_deselect(); return false; }
        uint16_t len = hdr[0] | ((uint16_t)hdr[1] << 8);
        cont = (len & 0x8000) != 0;
        len &= 0x7FFF;
        if (len == 0) { cs_deselect(); return false; }
        ch = hdr[2];
        uint16_t room = sizeof(dev->rx_buf) - total;
        uint16_t n = (len <= room) ? len : room;
        if (spi_rx(&dev->rx_buf[total], n) != HAL_OK) { cs_deselect(); return false; }
        cs_deselect();
        total += n;
        // If truncated, consume remaining bytes into a dummy sink
        if (n < len) {
            const uint16_t rem = len - n;
            uint8_t sink[16];
            uint16_t left = rem;
            cs_select();
            while (left) {
                uint16_t chunk = (left > sizeof(sink)) ? sizeof(sink) : left;
                if (spi_rx(sink, chunk) != HAL_OK) { cs_deselect(); return false; }
                left -= chunk;
            }
            cs_deselect();
        }
    } while (cont);
    *channel = ch;
    *payload_len = total;
    return true;
}

static bool write_packet(bno085_t* dev, uint8_t channel, const uint8_t* payload, uint16_t payload_len) {
    // Build header
    uint16_t len = payload_len + 4;
    dev->tx_buf[0] = (uint8_t)(len & 0xFF);
    dev->tx_buf[1] = (uint8_t)((len >> 8) & 0x7F); // no continuation
    dev->tx_buf[2] = channel;
    dev->tx_buf[3] = dev->tx_seq[channel]++;
    // Copy payload
    if (payload_len > 0 && payload != NULL) {
        memcpy(&dev->tx_buf[4], payload, payload_len);
    }
    cs_select();
    HAL_StatusTypeDef st = spi_tx(dev->tx_buf, payload_len + 4);
    cs_deselect();
    return (st == HAL_OK);
}

static void parse_reports(bno085_t* dev, const uint8_t* p, uint16_t n) {
    // Packets on REPORTS channel contain one or more reports: [report_id][...]
    uint16_t idx = 0;
    while (idx < n) {
        uint8_t rid = p[idx++];
        if (rid == SHTP_REPORT_ROTATION_VECTOR) {
            if (idx + 13 <= n) {
                // Layout per SH-2: status(1), i(2), j(2), k(2), real(2), accuracy(2), time(4)
                uint8_t status = p[idx++]; (void)status;
                int16_t qi = (int16_t)((p[idx] | (p[idx+1]<<8))); idx += 2;
                int16_t qj = (int16_t)((p[idx] | (p[idx+1]<<8))); idx += 2;
                int16_t qk = (int16_t)((p[idx] | (p[idx+1]<<8))); idx += 2;
                int16_t qr = (int16_t)((p[idx] | (p[idx+1]<<8))); idx += 2;
                uint16_t acc = (uint16_t)((p[idx] | (p[idx+1]<<8))); idx += 2; (void)acc;
                uint32_t ts = 0;
                if (idx + 4 <= n) { ts = (uint32_t)p[idx] | ((uint32_t)p[idx+1]<<8) | ((uint32_t)p[idx+2]<<16) | ((uint32_t)p[idx+3]<<24); idx += 4; }
                const float scale = 1.0f / 16384.0f; // Q14
                dev->quat.i = (float)qi * scale;
                dev->quat.j = (float)qj * scale;
                dev->quat.k = (float)qk * scale;
                dev->quat.real = (float)qr * scale;
                dev->quat.timestamp_us = ts; // sensor timestamp in us
                dev->quat.valid = true;
            } else {
                break;
            }
        } else {
            // Skip unknown report: they are TLV-like; simplest skip rest
            break;
        }
    }
}

bool BNO085_Begin(bno085_t* dev) {
    memset(dev, 0, sizeof(*dev));
    // Soft reset line has been released in BNO085_Reset
    // Read Product ID sequence via CONTROL channel request
    // Send "Get Feature Request" for base timestamp to sync
    // First, clear any pending packets by reading while INT is low
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin) == GPIO_PIN_RESET) {
        uint8_t ch; uint16_t n;
        if (!read_packet(dev, &ch, &n)) break;
    }

    // Enable Base Timestamp
    uint8_t ts_enable[9] = {
        SHTP_REPORT_SET_FEATURE_COMMAND,    // report ID
        SHTP_REPORT_BASE_TIMESTAMP,         // feature
        0x00, 0x00,                         // sequence nbr
        0x00, 0x00, 0x00, 0x00,             // interval (us) = 0 => on every report
        0x00
    };
    if (!write_packet(dev, SHTP_CHANNEL_CONTROL, ts_enable, sizeof(ts_enable))) return false;

    // Enable Rotation Vector at 100 Hz (interval 10,000 us)
    if (!BNO085_EnableRotationVector(dev, 10000)) return false;

    // Wait up to 500 ms for first report
    while ((HAL_GetTick() - t0) < 500) {
        (void)BNO085_Service(dev);
        if (dev->quat.valid) return true;
    }
    return false;
}

bool BNO085_EnableRotationVector(bno085_t* dev, uint16_t interval_us) {
    uint8_t payload[9];
    payload[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    payload[1] = SHTP_REPORT_ROTATION_VECTOR;
    payload[2] = 0x00; // seq
    payload[3] = 0x00;
    // interval as 32-bit little endian
    payload[4] = (uint8_t)(interval_us & 0xFF);
    payload[5] = (uint8_t)((interval_us >> 8) & 0xFF);
    payload[6] = 0x00;
    payload[7] = 0x00;
    payload[8] = 0x00; // dynamic range etc.
    return write_packet(dev, SHTP_CHANNEL_CONTROL, payload, sizeof(payload));
}

bool BNO085_Service(bno085_t* dev) {
    // If INT not asserted, nothing to read
    if (HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin) != GPIO_PIN_RESET) return false;
    uint8_t ch; uint16_t n;
    if (!read_packet(dev, &ch, &n)) return false;
    if (ch == SHTP_CHANNEL_REPORTS) {
        parse_reports(dev, dev->rx_buf, n);
        return true;
    }
    return false;
}

bool BNO085_GetQuaternion(bno085_t* dev, bno085_quat_t* out) {
    if (!dev->quat.valid) return false;
    *out = dev->quat;
    dev->quat.valid = false; // consume
    return true;
}
