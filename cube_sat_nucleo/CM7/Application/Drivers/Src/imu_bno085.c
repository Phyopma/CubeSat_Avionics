/*
 * imu_bno085.c
 */

#include "imu_bno085.h"
#include "config.h"
#include "serial_log_dma.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define SPI_TIMEOUT 100U

#define SCALE_Q14 (1.0f / 16384.0f)
#define SCALE_Q8  (1.0f / 256.0f)
#define SCALE_Q9  (1.0f / 512.0f)
#define SCALE_Q4  (1.0f / 16.0f)
#define GRAVITY_MPS2 9.80665f

static void cs_select(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static void cs_deselect(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef spi_rx(uint8_t *data, uint16_t size)
{
    return HAL_SPI_Receive(&hspi3, data, size, SPI_TIMEOUT);
}

static HAL_StatusTypeDef spi_tx(uint8_t *data, uint16_t size)
{
    return HAL_SPI_Transmit(&hspi3, data, size, SPI_TIMEOUT);
}

static bool read_packet(bno085_t *dev, uint8_t *channel, uint16_t *payload_len)
{
    uint16_t total_payload_received = 0U;
    uint8_t ch = 0U;
    bool continuation = false;

    (void)memset(dev->rx_buf, 0, sizeof(dev->rx_buf));

    do {
        uint8_t hdr[4];
        uint16_t fragment_len;
        uint16_t payload_in_fragment;

        cs_select();
        if (spi_rx(hdr, 4U) != HAL_OK) {
            cs_deselect();
            return false;
        }

        fragment_len = (uint16_t)((hdr[1] << 8) | hdr[0]);
        continuation = (fragment_len & 0x8000U) != 0U;
        fragment_len &= 0x7FFFU;
        if (fragment_len < 4U) {
            cs_deselect();
            if (fragment_len == 0U) {
                continue;
            }
            return false;
        }

        ch = hdr[2];
        payload_in_fragment = (uint16_t)(fragment_len - 4U);
        if (payload_in_fragment > 0U) {
            if ((uint32_t)total_payload_received + payload_in_fragment > sizeof(dev->rx_buf)) {
                uint16_t bytes_to_discard = payload_in_fragment;
                uint8_t sink[32];
                while (bytes_to_discard > 0U) {
                    uint16_t chunk = (bytes_to_discard > sizeof(sink)) ? (uint16_t)sizeof(sink) : bytes_to_discard;
                    if (spi_rx(sink, chunk) != HAL_OK) {
                        cs_deselect();
                        return false;
                    }
                    bytes_to_discard = (uint16_t)(bytes_to_discard - chunk);
                }
            } else {
                if (spi_rx(&dev->rx_buf[total_payload_received], payload_in_fragment) != HAL_OK) {
                    cs_deselect();
                    return false;
                }
                total_payload_received = (uint16_t)(total_payload_received + payload_in_fragment);
            }
        }
        cs_deselect();
    } while (continuation);

    *channel = ch;
    *payload_len = total_payload_received;
    return true;
}

static bool write_packet(bno085_t *dev, uint8_t channel, const uint8_t *payload, uint16_t payload_len)
{
    uint16_t len = (uint16_t)(payload_len + 4U);

    dev->tx_buf[0] = (uint8_t)(len & 0xFFU);
    dev->tx_buf[1] = (uint8_t)((len >> 8) & 0x7FU);
    dev->tx_buf[2] = channel;
    dev->tx_buf[3] = dev->tx_seq[channel]++;
    if (payload_len > 0U && payload != NULL) {
        memcpy(&dev->tx_buf[4], payload, payload_len);
    }

    cs_select();
    if (spi_tx(dev->tx_buf, len) != HAL_OK) {
        cs_deselect();
        return false;
    }
    cs_deselect();
    return true;
}

static bool BNO085_SetFeature(bno085_t *dev, uint8_t report_id, uint32_t interval_us)
{
    uint8_t payload[17];
    payload[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    payload[1] = report_id;
    payload[2] = 0x00U;
    payload[3] = 0x00U;
    payload[4] = 0x00U;
    payload[5] = (uint8_t)(interval_us & 0xFFU);
    payload[6] = (uint8_t)((interval_us >> 8) & 0xFFU);
    payload[7] = (uint8_t)((interval_us >> 16) & 0xFFU);
    payload[8] = (uint8_t)((interval_us >> 24) & 0xFFU);
    memset(&payload[9], 0, 8U);
    return write_packet(dev, SHTP_CHANNEL_CONTROL, payload, sizeof(payload));
}

static void bno085_copy_vec3(bno085_vec3_t *dst, float x, float y, float z,
                             uint8_t status, uint32_t local_ms, uint32_t sensor_us, uint32_t sample_seq)
{
    dst->x = x;
    dst->y = y;
    dst->z = z;
    dst->status = status;
    dst->accuracy = (uint8_t)(status & 0x03U);
    dst->local_timestamp_ms = local_ms;
    dst->sensor_timestamp_us = sensor_us;
    dst->sample_sequence = sample_seq;
    dst->valid = true;
}

static void bno085_copy_raw_vec3(bno085_raw_vec3_t *dst, int16_t x, int16_t y, int16_t z,
                                 uint8_t status, uint32_t local_ms, uint32_t sensor_us, uint32_t sample_seq)
{
    dst->x = x;
    dst->y = y;
    dst->z = z;
    dst->status = status;
    dst->accuracy = (uint8_t)(status & 0x03U);
    dst->local_timestamp_ms = local_ms;
    dst->sensor_timestamp_us = sensor_us;
    dst->sample_sequence = sample_seq;
    dst->valid = true;
}

static void bno085_copy_quat(bno085_quat_t *dst, int16_t qi, int16_t qj, int16_t qk, int16_t qr, int16_t accuracy,
                             uint8_t status, uint32_t local_ms, uint32_t sensor_us, uint32_t sample_seq)
{
    dst->i = (float)qi * SCALE_Q14;
    dst->j = (float)qj * SCALE_Q14;
    dst->k = (float)qk * SCALE_Q14;
    dst->real = (float)qr * SCALE_Q14;
    dst->accuracy_rad = (float)accuracy * SCALE_Q14;
    dst->status = status;
    dst->accuracy = (uint8_t)(status & 0x03U);
    dst->local_timestamp_ms = local_ms;
    dst->sensor_timestamp_us = sensor_us;
    dst->sample_sequence = sample_seq;
    dst->valid = true;
}

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

    {
        float gx = 2.0f * ((x * z) - (w * y));
        float gy = 2.0f * ((w * x) + (y * z));
        float gz = (w * w) - (x * x) - (y * y) + (z * z);
        *out_lin = dev->accel;
        out_lin->x = dev->accel.x - (gx * GRAVITY_MPS2);
        out_lin->y = dev->accel.y - (gy * GRAVITY_MPS2);
        out_lin->z = dev->accel.z - (gz * GRAVITY_MPS2);
        out_lin->valid = true;
    }
    return true;
}

static uint16_t bno085_report_len_for_id(uint8_t report_id)
{
    switch (report_id) {
        case SHTP_REPORT_BASE_TIMESTAMP:
        case SHTP_REPORT_TIMESTAMP_REBASE:
            return 5U;
        case SHTP_REPORT_ACCELEROMETER:
        case SHTP_REPORT_GYROSCOPE:
        case SHTP_REPORT_MAGNETIC_FIELD:
        case SHTP_REPORT_LINEAR_ACCELERATION:
        case SHTP_REPORT_GRAVITY:
            return 10U;
        case SHTP_REPORT_ROTATION_VECTOR:
        case SHTP_REPORT_GEOMAGNETIC_RV:
        case SHTP_REPORT_ARVR_STAB_RV:
            return 14U;
        case SHTP_REPORT_GAME_ROTATION_VECTOR:
        case SHTP_REPORT_ARVR_STAB_GRV:
            return 12U;
        case SHTP_REPORT_GYROSCOPE_UNCAL:
        case SHTP_REPORT_MAGNETIC_FIELD_UNCAL:
        case SHTP_REPORT_RAW_ACCELEROMETER:
        case SHTP_REPORT_RAW_GYROSCOPE:
        case SHTP_REPORT_RAW_MAGNETOMETER:
            return 16U;
        case SHTP_REPORT_SET_FEATURE_COMMAND:
            return 17U;
        case SHTP_REPORT_PRODUCT_ID:
            return 16U;
        default:
            return 0U;
    }
}

static void parse_control_report(bno085_t *dev, const uint8_t *payload, uint16_t len)
{
    if (len >= 17U && payload[0] == 0xFCU) {
        uint8_t feature_id = payload[1];
        uint32_t interval_us =
            ((uint32_t)payload[8] << 24) |
            ((uint32_t)payload[7] << 16) |
            ((uint32_t)payload[6] << 8) |
            (uint32_t)payload[5];
        dev->feature_state[feature_id].enabled = (interval_us != 0U);
        dev->feature_state[feature_id].acknowledged = true;
        dev->feature_state[feature_id].interval_us = interval_us;
        dev->feature_state[feature_id].last_update_ms = HAL_GetTick();
        BNO085_Log("BNO085 feature rsp id=0x%02X interval_us=%lu\r\n",
                   feature_id, (unsigned long)interval_us);
    }
}

static void parse_reports(bno085_t *dev, const uint8_t *p, uint16_t n, uint8_t channel)
{
    static uint8_t seen_report_id[256] = {0};
    uint16_t idx = 0U;
    while (idx < n) {
        uint8_t report_id = p[idx];
        uint16_t report_len = bno085_report_len_for_id(report_id);

        if (seen_report_id[report_id] == 0U) {
            seen_report_id[report_id] = 1U;
            log_printf_dma("BNO085 report id seen ch=%u id=0x%02X", channel, report_id);
        }

        if (report_len == 0U || (uint32_t)idx + report_len > n) {
            static uint8_t seen_bad_id[256] = {0};
            if (seen_bad_id[report_id] == 0U) {
                seen_bad_id[report_id] = 1U;
                log_printf_dma("WARN BNO085 unknown/truncated report ch=%u id=0x%02X n=%u idx=%u",
                               channel, report_id, (unsigned)n, (unsigned)idx);
            }
            break;
        }

        if (report_id == SHTP_REPORT_BASE_TIMESTAMP || report_id == SHTP_REPORT_TIMESTAMP_REBASE) {
            dev->last_base_timestamp_us =
                ((uint32_t)p[idx + 4U] << 24) |
                ((uint32_t)p[idx + 3U] << 16) |
                ((uint32_t)p[idx + 2U] << 8) |
                (uint32_t)p[idx + 1U];
            idx = (uint16_t)(idx + report_len);
            continue;
        }

        {
            uint8_t status = p[idx + 2U];
            uint8_t sample_seq = p[idx + 1U];
            uint16_t data_idx = (uint16_t)(idx + 4U);
            uint32_t local_timestamp_ms = HAL_GetTick();
            uint32_t sensor_timestamp_us = dev->last_base_timestamp_us;

            dev->last_local_timestamp_ms = local_timestamp_ms;

            switch (report_id) {
                case SHTP_REPORT_ROTATION_VECTOR:
                    bno085_copy_quat(
                        &dev->quat,
                        (int16_t)((p[data_idx + 1U] << 8) | p[data_idx]),
                        (int16_t)((p[data_idx + 3U] << 8) | p[data_idx + 2U]),
                        (int16_t)((p[data_idx + 5U] << 8) | p[data_idx + 4U]),
                        (int16_t)((p[data_idx + 7U] << 8) | p[data_idx + 6U]),
                        (int16_t)((p[data_idx + 9U] << 8) | p[data_idx + 8U]),
                        status,
                        local_timestamp_ms,
                        sensor_timestamp_us,
                        sample_seq);
                    break;
                case SHTP_REPORT_GAME_ROTATION_VECTOR:
                    bno085_copy_quat(
                        &dev->game_quat,
                        (int16_t)((p[data_idx + 1U] << 8) | p[data_idx]),
                        (int16_t)((p[data_idx + 3U] << 8) | p[data_idx + 2U]),
                        (int16_t)((p[data_idx + 5U] << 8) | p[data_idx + 4U]),
                        (int16_t)((p[data_idx + 7U] << 8) | p[data_idx + 6U]),
                        0,
                        status,
                        local_timestamp_ms,
                        sensor_timestamp_us,
                        sample_seq);
                    break;
                case SHTP_REPORT_LINEAR_ACCELERATION:
                    bno085_copy_vec3(
                        &dev->lin_accel,
                        (float)(int16_t)((p[data_idx + 1U] << 8) | p[data_idx]) * SCALE_Q8,
                        (float)(int16_t)((p[data_idx + 3U] << 8) | p[data_idx + 2U]) * SCALE_Q8,
                        (float)(int16_t)((p[data_idx + 5U] << 8) | p[data_idx + 4U]) * SCALE_Q8,
                        status,
                        local_timestamp_ms,
                        sensor_timestamp_us,
                        sample_seq);
                    break;
                case SHTP_REPORT_ACCELEROMETER:
                    bno085_copy_vec3(
                        &dev->accel,
                        (float)(int16_t)((p[data_idx + 1U] << 8) | p[data_idx]) * SCALE_Q8,
                        (float)(int16_t)((p[data_idx + 3U] << 8) | p[data_idx + 2U]) * SCALE_Q8,
                        (float)(int16_t)((p[data_idx + 5U] << 8) | p[data_idx + 4U]) * SCALE_Q8,
                        status,
                        local_timestamp_ms,
                        sensor_timestamp_us,
                        sample_seq);
                    break;
                case SHTP_REPORT_GRAVITY:
                    bno085_copy_vec3(
                        &dev->gravity,
                        (float)(int16_t)((p[data_idx + 1U] << 8) | p[data_idx]) * SCALE_Q8,
                        (float)(int16_t)((p[data_idx + 3U] << 8) | p[data_idx + 2U]) * SCALE_Q8,
                        (float)(int16_t)((p[data_idx + 5U] << 8) | p[data_idx + 4U]) * SCALE_Q8,
                        status,
                        local_timestamp_ms,
                        sensor_timestamp_us,
                        sample_seq);
                    break;
                case SHTP_REPORT_GYROSCOPE:
                    bno085_copy_vec3(
                        &dev->gyro,
                        (float)(int16_t)((p[data_idx + 1U] << 8) | p[data_idx]) * SCALE_Q9,
                        (float)(int16_t)((p[data_idx + 3U] << 8) | p[data_idx + 2U]) * SCALE_Q9,
                        (float)(int16_t)((p[data_idx + 5U] << 8) | p[data_idx + 4U]) * SCALE_Q9,
                        status,
                        local_timestamp_ms,
                        sensor_timestamp_us,
                        sample_seq);
                    break;
                case SHTP_REPORT_MAGNETIC_FIELD:
                    bno085_copy_vec3(
                        &dev->mag_cal,
                        (float)(int16_t)((p[data_idx + 1U] << 8) | p[data_idx]) * SCALE_Q4,
                        (float)(int16_t)((p[data_idx + 3U] << 8) | p[data_idx + 2U]) * SCALE_Q4,
                        (float)(int16_t)((p[data_idx + 5U] << 8) | p[data_idx + 4U]) * SCALE_Q4,
                        status,
                        local_timestamp_ms,
                        sensor_timestamp_us,
                        sample_seq);
                    break;
                case SHTP_REPORT_MAGNETIC_FIELD_UNCAL:
                    bno085_copy_vec3(
                        &dev->mag_uncal,
                        (float)(int16_t)((p[data_idx + 1U] << 8) | p[data_idx]) * SCALE_Q4,
                        (float)(int16_t)((p[data_idx + 3U] << 8) | p[data_idx + 2U]) * SCALE_Q4,
                        (float)(int16_t)((p[data_idx + 5U] << 8) | p[data_idx + 4U]) * SCALE_Q4,
                        status,
                        local_timestamp_ms,
                        sensor_timestamp_us,
                        sample_seq);
                    break;
                case SHTP_REPORT_RAW_MAGNETOMETER:
                    sensor_timestamp_us =
                        ((uint32_t)p[idx + 15U] << 24) |
                        ((uint32_t)p[idx + 14U] << 16) |
                        ((uint32_t)p[idx + 13U] << 8) |
                        (uint32_t)p[idx + 12U];
                    bno085_copy_raw_vec3(
                        &dev->mag_raw,
                        (int16_t)((p[data_idx + 1U] << 8) | p[data_idx]),
                        (int16_t)((p[data_idx + 3U] << 8) | p[data_idx + 2U]),
                        (int16_t)((p[data_idx + 5U] << 8) | p[data_idx + 4U]),
                        status,
                        local_timestamp_ms,
                        sensor_timestamp_us,
                        sample_seq);
                    break;
                default:
                    break;
            }

            if (dev->lin_accel.valid == false && dev->accel.valid == true) {
                if (dev->gravity.valid == true) {
                    dev->lin_accel = dev->accel;
                    dev->lin_accel.x = dev->accel.x - dev->gravity.x;
                    dev->lin_accel.y = dev->accel.y - dev->gravity.y;
                    dev->lin_accel.z = dev->accel.z - dev->gravity.z;
                    dev->lin_accel.valid = true;
                } else {
                    (void)bno085_try_derive_lin_from_quat(dev, &dev->lin_accel);
                }
            }
        }

        idx = (uint16_t)(idx + report_len);
    }
}

void BNO085_Reset(void)
{
    BNO085_Log("BNO085_Reset: Toggling RST pin...\r\n");
    cs_deselect();
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(15U);
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50U);
}

bool BNO085_WaitForAck(bno085_t *dev)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 500U) {
        uint8_t ch;
        if (BNO085_Service(dev, &ch) && ch == SHTP_CHANNEL_CONTROL) {
            return true;
        }
    }
    return false;
}

bool BNO085_Begin(bno085_t *dev)
{
    uint32_t t0;
    bool saw_packet = false;
    memset(dev, 0, sizeof(*dev));
    BNO085_Reset();

    BNO085_Log("Waiting for BNO085 to boot...\r\n");
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 2000U) {
        uint8_t ch = 0U;
        if (BNO085_Service(dev, &ch)) {
            saw_packet = true;
        }
        HAL_Delay(1U);
    }
    if (!saw_packet) {
        BNO085_Log("WARN no boot packets observed from BNO085\r\n");
    }
    return saw_packet;
}

bool BNO085_ConfigureFeature(bno085_t *dev, uint8_t report_id, uint32_t interval_us)
{
    if (!BNO085_SetFeature(dev, report_id, interval_us)) {
        return false;
    }
    dev->feature_state[report_id].enabled = (interval_us != 0U);
    dev->feature_state[report_id].acknowledged = false;
    dev->feature_state[report_id].interval_us = interval_us;
    dev->feature_state[report_id].last_update_ms = HAL_GetTick();
    (void)BNO085_RequestFeatureState(dev, report_id);
    return true;
}

bool BNO085_RequestFeatureState(bno085_t *dev, uint8_t report_id)
{
    uint8_t payload[2];
    payload[0] = 0xFEU;
    payload[1] = report_id;
    return write_packet(dev, SHTP_CHANNEL_CONTROL, payload, sizeof(payload));
}
bool BNO085_EnableMagnetometerUncal(bno085_t* dev, uint32_t interval_us) {
    return BNO085_SetFeature(dev, SHTP_REPORT_MAGNETIC_FIELD_UNCAL, interval_us);
}

bool BNO085_GetFeatureState(const bno085_t *dev, uint8_t report_id, bno085_feature_state_t *out)
{
    if (dev == NULL || out == NULL) {
        return false;
    }
    *out = dev->feature_state[report_id];
    return true;
}

bool BNO085_EnableRotationVector(bno085_t *dev, uint32_t interval_us) { return BNO085_ConfigureFeature(dev, SHTP_REPORT_ROTATION_VECTOR, interval_us); }
bool BNO085_EnableGameRotationVector(bno085_t *dev, uint32_t interval_us) { return BNO085_ConfigureFeature(dev, SHTP_REPORT_GAME_ROTATION_VECTOR, interval_us); }
bool BNO085_EnableLinearAccelerometer(bno085_t *dev, uint32_t interval_us) { return BNO085_ConfigureFeature(dev, SHTP_REPORT_LINEAR_ACCELERATION, interval_us); }
bool BNO085_EnableAccelerometer(bno085_t *dev, uint32_t interval_us) { return BNO085_ConfigureFeature(dev, SHTP_REPORT_ACCELEROMETER, interval_us); }
bool BNO085_EnableGravity(bno085_t *dev, uint32_t interval_us) { return BNO085_ConfigureFeature(dev, SHTP_REPORT_GRAVITY, interval_us); }
bool BNO085_EnableGyroscope(bno085_t *dev, uint32_t interval_us) { return BNO085_ConfigureFeature(dev, SHTP_REPORT_GYROSCOPE, interval_us); }
bool BNO085_EnableMagnetometer(bno085_t *dev, uint32_t interval_us) { return BNO085_ConfigureFeature(dev, SHTP_REPORT_MAGNETIC_FIELD, interval_us); }
bool BNO085_EnableMagnetometerUncal(bno085_t *dev, uint32_t interval_us) { return BNO085_ConfigureFeature(dev, SHTP_REPORT_MAGNETIC_FIELD_UNCAL, interval_us); }
bool BNO085_EnableRawMagnetometer(bno085_t *dev, uint32_t interval_us) { return BNO085_ConfigureFeature(dev, SHTP_REPORT_RAW_MAGNETOMETER, interval_us); }

bool BNO085_Service(bno085_t *dev, uint8_t *channel_read)
{
    uint8_t ch;
    uint16_t n;

    if (HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin) != GPIO_PIN_RESET) {
        return false;
    }

    if (!read_packet(dev, &ch, &n)) {
        return false;
    }

    if (channel_read != NULL) {
        *channel_read = ch;
    }

    if (ch == SHTP_CHANNEL_CONTROL) {
        parse_control_report(dev, dev->rx_buf, n);
    } else if (ch == SHTP_CHANNEL_REPORTS ||
               ch == SHTP_CHANNEL_WAKE_REPORTS ||
               ch == SHTP_CHANNEL_GYRO_ROTATION_VECTOR) {
        parse_reports(dev, dev->rx_buf, n, ch);
    }

    return true;
}

bool BNO085_CopyQuaternion(const bno085_t *dev, bno085_quat_t *out)
{
    if (dev == NULL || out == NULL || !dev->quat.valid) {
        return false;
    }
    *out = dev->quat;
    return true;
}

bool BNO085_CopyGameQuaternion(const bno085_t *dev, bno085_quat_t *out)
{
    if (dev == NULL || out == NULL || !dev->game_quat.valid) {
        return false;
    }
    *out = dev->game_quat;
    return true;
}

bool BNO085_CopyLinearAcceleration(const bno085_t *dev, bno085_vec3_t *out)
{
    if (dev == NULL || out == NULL || !dev->lin_accel.valid) {
        return false;
    }
    *out = dev->lin_accel;
    return true;
}

bool BNO085_CopyAccelerometer(const bno085_t *dev, bno085_vec3_t *out)
{
    if (dev == NULL || out == NULL || !dev->accel.valid) {
        return false;
    }
    *out = dev->accel;
    return true;
}

bool BNO085_CopyGravity(const bno085_t *dev, bno085_vec3_t *out)
{
    if (dev == NULL || out == NULL || !dev->gravity.valid) {
        return false;
    }
    *out = dev->gravity;
    return true;
}

bool BNO085_CopyGyroscope(const bno085_t *dev, bno085_vec3_t *out)
{
    if (dev == NULL || out == NULL || !dev->gyro.valid) {
        return false;
    }
    *out = dev->gyro;
    return true;
}

bool BNO085_CopyMagnetometerCal(const bno085_t *dev, bno085_vec3_t *out)
{
    if (dev == NULL || out == NULL || !dev->mag_cal.valid) {
        return false;
    }
    *out = dev->mag_cal;
    return true;
}

bool BNO085_CopyMagnetometerUncal(const bno085_t *dev, bno085_vec3_t *out)
{
    if (dev == NULL || out == NULL || !dev->mag_uncal.valid) {
        return false;
    }
    *out = dev->mag_uncal;
    return true;
}

bool BNO085_CopyRawMagnetometer(const bno085_t *dev, bno085_raw_vec3_t *out)
{
    if (dev == NULL || out == NULL || !dev->mag_raw.valid) {
        return false;
    }
    *out = dev->mag_raw;
    return true;
}

void BNO085_Log(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    if (vsnprintf(buf, sizeof(buf), fmt, args) > 0) {
        log_printf_dma("%s", buf);
    }
    va_end(args);
}
