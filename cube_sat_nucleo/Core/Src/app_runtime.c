#include "app_runtime.h"

#include "main.h"
#include "config.h"
#include "imu_runtime.h"
#include "power_runtime.h"
#include "inner_loop_control.h"
#include "outer_loop_control.h"
#include "math_lib.h"
#include "current_sensor.h"
#include "adt7420.h"
#include "imu_bno085.h"
#include "teleplot.h"
#include "serial_log_dma.h"
#include "i2c.h"
#include "usart.h"
#include "tim.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define ADCS_PERIOD_MS 10U
#define TELEMETRY_PERIOD_MS 50U
#define POWER_SAMPLE_PERIOD_MS SENSOR_SAMPLE_PERIOD_MS
#define SENSOR_LOG_PERIOD_MS SENSOR_LOG_PRINT_PERIOD_MS
#define IMU_SERVICE_PERIOD_MS 2U
#define SENSOR_INIT_RETRY_MS 3000U
#define IMU_RECONCILE_PERIOD_MS 100U
#define IMU_SERVICE_BUDGET 16U
#define MTQ_FIRING_CURRENT_THRESHOLD_A 0.001f

TaskHandle_t g_adcs_task_handle = NULL;

extern bno085_t imu;

static ADT7420_Handle g_temp_sensor;
static uint8_t g_temp_sensor_ready = 0U;
static uint8_t g_imu_ready = 0U;
static uint32_t g_last_temp_init_attempt_ms = 0U;
static uint32_t g_last_imu_init_attempt_ms = 0U;
static uint32_t g_last_imu_reconcile_ms = 0U;
static imu_report_id_t g_next_reconcile_report = IMU_REPORT_ROTATION_VECTOR;
static float g_power_avg_current_mA = 0.0f;
static float g_power_avg_voltage_mV = 0.0f;
static uint8_t g_power_avg_valid = 0U;

static void AppRuntime_BootPrint(const char *msg)
{
    if (msg == NULL) {
        return;
    }
    {
        size_t n = strnlen(msg, 120U);
        static uint8_t crlf[2] = {'\r', '\n'};
        if (n == 0U) {
            return;
        }
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)n, 20U);
        (void)HAL_UART_Transmit(&huart2, crlf, 2U, 20U);
    }
}

static void AppRuntime_FatalStartup(const char *msg)
{
    if (msg != NULL) {
        char line[96];
        int n = snprintf(line, sizeof(line), "FATAL RTOS startup: %s", msg);
        if (n > 0) {
            AppRuntime_BootPrint(line);
        }
        log_printf_dma("FATAL RTOS startup: %s", msg);
        serial_log_process_tx();
    }
    Error_Handler();
}

static uint32_t AppRuntime_ReportAgeMs(uint32_t now_ms, const imu_report_meta_t *meta)
{
    if (meta == NULL || meta->valid == 0U) {
        return UINT32_MAX;
    }
    return (uint32_t)(now_ms - meta->local_timestamp_ms);
}

static int16_t AppRuntime_ClampInt16(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static uint16_t AppRuntime_ClampUInt16(int32_t value)
{
    if (value > UINT16_MAX) {
        return UINT16_MAX;
    }
    if (value < 0) {
        return 0U;
    }
    return (uint16_t)value;
}

static int32_t AppRuntime_RoundToInt32(float value)
{
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void AppRuntime_CopyQuatSlot(imu_quat_slot_t *dst, bool enabled, const bno085_quat_t *src)
{
    memset(dst, 0, sizeof(*dst));
    dst->meta.enabled = enabled ? 1U : 0U;
    if (src == NULL || src->valid == false) {
        return;
    }
    dst->meta.valid = 1U;
    dst->meta.status = src->status;
    dst->meta.accuracy = src->accuracy;
    dst->meta.local_timestamp_ms = src->local_timestamp_ms;
    dst->meta.sensor_timestamp_us = src->sensor_timestamp_us;
    dst->meta.sample_sequence = src->sample_sequence;
    dst->value.w = src->real;
    dst->value.x = src->i;
    dst->value.y = src->j;
    dst->value.z = src->k;
    dst->value.accuracy_rad = src->accuracy_rad;
}

static void AppRuntime_CopyVec3Slot(imu_vec3_slot_t *dst, bool enabled, const bno085_vec3_t *src)
{
    memset(dst, 0, sizeof(*dst));
    dst->meta.enabled = enabled ? 1U : 0U;
    if (src == NULL || src->valid == false) {
        return;
    }
    dst->meta.valid = 1U;
    dst->meta.status = src->status;
    dst->meta.accuracy = src->accuracy;
    dst->meta.local_timestamp_ms = src->local_timestamp_ms;
    dst->meta.sensor_timestamp_us = src->sensor_timestamp_us;
    dst->meta.sample_sequence = src->sample_sequence;
    dst->value.x = src->x;
    dst->value.y = src->y;
    dst->value.z = src->z;
}

static void AppRuntime_CopyRawVec3Slot(imu_raw_vec3_slot_t *dst, bool enabled, const bno085_raw_vec3_t *src)
{
    memset(dst, 0, sizeof(*dst));
    dst->meta.enabled = enabled ? 1U : 0U;
    if (src == NULL || src->valid == false) {
        return;
    }
    dst->meta.valid = 1U;
    dst->meta.status = src->status;
    dst->meta.accuracy = src->accuracy;
    dst->meta.local_timestamp_ms = src->local_timestamp_ms;
    dst->meta.sensor_timestamp_us = src->sensor_timestamp_us;
    dst->meta.sample_sequence = src->sample_sequence;
    dst->value.x = src->x;
    dst->value.y = src->y;
    dst->value.z = src->z;
}

static void AppRuntime_PublishImuSnapshot(uint32_t now_ms)
{
    imu_snapshot_t snapshot;
    imu_request_set_t resolved;
    bno085_quat_t quat;
    bno085_quat_t game_quat;
    bno085_vec3_t gyro;
    bno085_vec3_t mag_cal;
    bno085_vec3_t mag_uncal;
    bno085_raw_vec3_t mag_raw;
    bno085_vec3_t lin_accel;
    bno085_vec3_t accel;
    bno085_vec3_t gravity;

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.publish_timestamp_ms = now_ms;
    IMU_CopyResolvedRequestSet(&resolved);

    memset(&quat, 0, sizeof(quat));
    memset(&game_quat, 0, sizeof(game_quat));
    memset(&gyro, 0, sizeof(gyro));
    memset(&mag_cal, 0, sizeof(mag_cal));
    memset(&mag_uncal, 0, sizeof(mag_uncal));
    memset(&mag_raw, 0, sizeof(mag_raw));
    memset(&lin_accel, 0, sizeof(lin_accel));
    memset(&accel, 0, sizeof(accel));
    memset(&gravity, 0, sizeof(gravity));

    (void)BNO085_CopyQuaternion(&imu, &quat);
    (void)BNO085_CopyGameQuaternion(&imu, &game_quat);
    (void)BNO085_CopyGyroscope(&imu, &gyro);
    (void)BNO085_CopyMagnetometerCal(&imu, &mag_cal);
    (void)BNO085_CopyMagnetometerUncal(&imu, &mag_uncal);
    (void)BNO085_CopyRawMagnetometer(&imu, &mag_raw);
    (void)BNO085_CopyLinearAcceleration(&imu, &lin_accel);
    (void)BNO085_CopyAccelerometer(&imu, &accel);
    (void)BNO085_CopyGravity(&imu, &gravity);

    AppRuntime_CopyQuatSlot(&snapshot.rotation_vector, resolved.reports[IMU_REPORT_ROTATION_VECTOR].enabled != 0U, &quat);
    AppRuntime_CopyQuatSlot(&snapshot.game_rotation_vector, resolved.reports[IMU_REPORT_GAME_ROTATION_VECTOR].enabled != 0U, &game_quat);
    AppRuntime_CopyVec3Slot(&snapshot.gyro, resolved.reports[IMU_REPORT_GYRO].enabled != 0U, &gyro);
    AppRuntime_CopyVec3Slot(&snapshot.magnetometer_cal, resolved.reports[IMU_REPORT_MAG_CAL].enabled != 0U, &mag_cal);
    AppRuntime_CopyVec3Slot(&snapshot.magnetometer_uncal, resolved.reports[IMU_REPORT_MAG_UNCAL].enabled != 0U, &mag_uncal);
    AppRuntime_CopyRawVec3Slot(&snapshot.magnetometer_raw, resolved.reports[IMU_REPORT_MAG_RAW].enabled != 0U, &mag_raw);
    AppRuntime_CopyVec3Slot(&snapshot.linear_accel, resolved.reports[IMU_REPORT_LINEAR_ACCEL].enabled != 0U, &lin_accel);
    AppRuntime_CopyVec3Slot(&snapshot.accel, resolved.reports[IMU_REPORT_ACCEL].enabled != 0U, &accel);
    AppRuntime_CopyVec3Slot(&snapshot.gravity, resolved.reports[IMU_REPORT_GRAVITY].enabled != 0U, &gravity);

    IMU_PublishSnapshot(&snapshot);
}

static void AppRuntime_UpdateAppliedRequests(void)
{
    imu_report_id_t report_id;
    for (report_id = 0; report_id < IMU_REPORT_COUNT; ++report_id) {
        bno085_feature_state_t feature_state;
        if (!BNO085_GetFeatureState(&imu, IMU_ReportToFeatureId(report_id), &feature_state)) {
            continue;
        }
        if (feature_state.acknowledged) {
            IMU_SetAppliedRequest(report_id, feature_state.enabled, feature_state.interval_us);
        }
    }
}

static void AppRuntime_ReconcileImuRequests(uint32_t now_ms)
{
    imu_request_set_t desired;
    uint32_t attempt;

    if (g_imu_ready == 0U) {
        return;
    }

    AppRuntime_UpdateAppliedRequests();
    if ((uint32_t)(now_ms - g_last_imu_reconcile_ms) < IMU_RECONCILE_PERIOD_MS) {
        return;
    }

    IMU_CopyResolvedRequestSet(&desired);
    for (attempt = 0; attempt < IMU_REPORT_COUNT; ++attempt) {
        imu_report_id_t report_id = (imu_report_id_t)((g_next_reconcile_report + attempt) % IMU_REPORT_COUNT);
        imu_request_entry_t desired_entry = desired.reports[report_id];
        bno085_feature_state_t feature_state;
        bool desired_enabled;

        (void)BNO085_GetFeatureState(&imu, IMU_ReportToFeatureId(report_id), &feature_state);
        desired_enabled = (desired_entry.enabled != 0U) && (desired_entry.interval_us != 0U);

        if (feature_state.acknowledged &&
            feature_state.enabled == desired_enabled &&
            (desired_enabled == false || feature_state.interval_us == desired_entry.interval_us)) {
            continue;
        }

        if (BNO085_ConfigureFeature(&imu,
                                    IMU_ReportToFeatureId(report_id),
                                    desired_enabled ? desired_entry.interval_us : 0U)) {
            log_printf_dma("IMU request %s enabled=%u interval_us=%lu",
                           IMU_ReportName(report_id),
                           desired_enabled ? 1U : 0U,
                           (unsigned long)(desired_enabled ? desired_entry.interval_us : 0U));
        }

        g_last_imu_reconcile_ms = now_ms;
        g_next_reconcile_report = (imu_report_id_t)((report_id + 1U) % IMU_REPORT_COUNT);
        break;
    }
}

static void app_control_step(void)
{
    InnerLoop_Update();
}

static void app_sensor_step(void)
{
    uint32_t now_ms = HAL_GetTick();
    power_snapshot_t power_snapshot;
    float current_amps = 0.0f;
    float bus_voltage_volts = 0.0f;
    float temp_c = 0.0f;
    uint8_t temp_status = 0U;
    uint32_t current_age_ms = UINT32_MAX;
    uint32_t temp_age_ms = UINT32_MAX;

    CurrentSensor_RunAsyncSample();
    CurrentSensor_SubmitSampleRequest();

    if (g_temp_sensor_ready == 0U) {
        if ((uint32_t)(now_ms - g_last_temp_init_attempt_ms) >= SENSOR_INIT_RETRY_MS) {
            g_last_temp_init_attempt_ms = now_ms;
            if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(ADT7420_I2C_ADDR_7BIT << 1), 2U, 20U) == HAL_OK &&
                ADT7420_Init(&g_temp_sensor, &hi2c2, ADT7420_I2C_ADDR_7BIT) == HAL_OK) {
                ADT7420_SubmitSampleRequest();
                g_temp_sensor_ready = 1U;
                log_printf_dma("ADT7420 init OK");
            }
        }
    } else {
        ADT7420_RunAsyncSample(&g_temp_sensor);
        ADT7420_SubmitSampleRequest();
    }

    (void)memset(&power_snapshot, 0, sizeof(power_snapshot));
    power_snapshot.publish_timestamp_ms = now_ms;
    power_snapshot.source_age_ms = UINT32_MAX;

    if (CurrentSensor_GetLatestPowerSample(&current_amps, &bus_voltage_volts, &current_age_ms, now_ms) != 0) {
        float current_mA = current_amps * 1000.0f;
        float voltage_mV = bus_voltage_volts * 1000.0f;
        if (g_power_avg_valid == 0U) {
            g_power_avg_current_mA = current_mA;
            g_power_avg_voltage_mV = voltage_mV;
            g_power_avg_valid = 1U;
        } else {
            g_power_avg_current_mA = (g_power_avg_current_mA * 0.90f) + (current_mA * 0.10f);
            g_power_avg_voltage_mV = (g_power_avg_voltage_mV * 0.90f) + (voltage_mV * 0.10f);
        }
        power_snapshot.battery.current_mA = AppRuntime_ClampInt16(AppRuntime_RoundToInt32(current_mA));
        power_snapshot.battery.average_current_mA = AppRuntime_ClampInt16(AppRuntime_RoundToInt32(g_power_avg_current_mA));
        power_snapshot.battery.voltage_mV = AppRuntime_ClampUInt16(AppRuntime_RoundToInt32(voltage_mV));
        power_snapshot.battery.average_voltage_mV = AppRuntime_ClampUInt16(AppRuntime_RoundToInt32(g_power_avg_voltage_mV));
        power_snapshot.battery.valid_mask |= POWER_BATTERY_VALID_CURRENT |
                                             POWER_BATTERY_VALID_AVERAGE_CURRENT |
                                             POWER_BATTERY_VALID_VOLTAGE |
                                             POWER_BATTERY_VALID_AVERAGE_VOLTAGE;
        power_snapshot.source_age_ms = current_age_ms;
    }

    if (ADT7420_GetLatestSampleWithStatus(&temp_c, &temp_status, &temp_age_ms, now_ms) != 0) {
        power_snapshot.digital_temp.temperature_cC = AppRuntime_ClampInt16(AppRuntime_RoundToInt32(temp_c * 100.0f));
        power_snapshot.digital_temp.status = temp_status;
        power_snapshot.digital_temp.valid_mask |= POWER_DIGITAL_TEMP_VALID_TEMPERATURE | POWER_DIGITAL_TEMP_VALID_STATUS;
        if (temp_age_ms < power_snapshot.source_age_ms) {
            power_snapshot.source_age_ms = temp_age_ms;
        }
    }

    Power_PublishSnapshot(&power_snapshot);
}

static void app_telemetry_step(void)
{
#if HARDWARE_TELEPLOT_ENABLE
    mtq_state_t data;
    power_snapshot_t power;
    if (!InnerLoop_GetStateSnapshot(&data, 0U)) {
        return;
    }
    Teleplot_Update("TargetX", data.target_current_x * 1000.0f);
    Teleplot_Update("TargetY", data.target_current_y * 1000.0f);
    Teleplot_Update("TargetZ", data.target_current_z * 1000.0f);
    Teleplot_Update("CurrentX", data.measured_current_x * 1000.0f);
    Teleplot_Update("CurrentY", data.measured_current_y * 1000.0f);
    Teleplot_Update("CurrentZ", data.measured_current_z * 1000.0f);
    Teleplot_Update("VoltX", data.command_voltage_x);
    Teleplot_Update("VoltY", data.command_voltage_y);
    Teleplot_Update("VoltZ", data.command_voltage_z);
    Power_ReadSnapshot(&power);
    if ((power.battery.valid_mask & POWER_BATTERY_VALID_CURRENT) != 0U) {
        Teleplot_Update("Ibat", (float)power.battery.current_mA);
    }
    if ((power.battery.valid_mask & POWER_BATTERY_VALID_VOLTAGE) != 0U) {
        Teleplot_Update("Vbat", (float)power.battery.voltage_mV);
    }
    if ((power.digital_temp.valid_mask & POWER_DIGITAL_TEMP_VALID_TEMPERATURE) != 0U) {
        Teleplot_Update("TempC", (float)power.digital_temp.temperature_cC * 0.01f);
    }
#endif
}

static void app_imu_step(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t serviced = 0U;

    if (g_imu_ready == 0U) {
        if ((uint32_t)(now_ms - g_last_imu_init_attempt_ms) >= SENSOR_INIT_RETRY_MS) {
            g_last_imu_init_attempt_ms = now_ms;
            if (BNO085_Begin(&imu)) {
                g_imu_ready = 1U;
                log_printf_dma("BNO085 init OK");
                g_last_imu_reconcile_ms = now_ms - IMU_RECONCILE_PERIOD_MS;
            } else {
                log_printf_dma("WARN BNO085 init retry failed");
            }
        }
        return;
    }

    while (serviced < IMU_SERVICE_BUDGET) {
        if (!BNO085_Service(&imu, NULL)) {
            break;
        }
        serviced += 1U;
    }

    AppRuntime_ReconcileImuRequests(now_ms);
    AppRuntime_PublishImuSnapshot(now_ms);
}

static bool AppRuntime_IsMtqFiring(const mtq_state_t *mtq_state)
{
    if (mtq_state == NULL) {
        return false;
    }
    return (fabsf(mtq_state->target_current_x) > MTQ_FIRING_CURRENT_THRESHOLD_A) ||
           (fabsf(mtq_state->target_current_y) > MTQ_FIRING_CURRENT_THRESHOLD_A) ||
           (fabsf(mtq_state->target_current_z) > MTQ_FIRING_CURRENT_THRESHOLD_A);
}

static quat_t AppRuntime_SelectOrientation(const imu_snapshot_t *snapshot, adcs_mode_t mode, bool mtq_firing)
{
    quat_t selected = {1.0f, 0.0f, 0.0f, 0.0f};
    const imu_quat_slot_t *primary = NULL;
    const imu_quat_slot_t *fallback = NULL;

    if (snapshot == NULL) {
        return selected;
    }

    if (mtq_firing) {
        primary = &snapshot->game_rotation_vector;
        fallback = &snapshot->rotation_vector;
    } else if (mode == CTRL_MODE_POINTING) {
        primary = &snapshot->rotation_vector;
        fallback = &snapshot->game_rotation_vector;
    } else {
        primary = &snapshot->game_rotation_vector;
        fallback = &snapshot->rotation_vector;
    }

    if (primary->meta.valid == 0U && fallback->meta.valid != 0U) {
        primary = fallback;
    }
    if (primary->meta.valid == 0U) {
        return selected;
    }

    selected.w = primary->value.w;
    selected.x = primary->value.x;
    selected.y = primary->value.y;
    selected.z = primary->value.z;
    return selected;
}

static vec3_t AppRuntime_SelectMagneticFieldTesla(const imu_snapshot_t *snapshot, bool mtq_firing)
{
    const float microtesla_to_tesla = 1.0e-6f;
    vec3_t field = {0.0f, 0.0f, 0.0f};
    const imu_vec3_slot_t *selected = NULL;

    if (snapshot == NULL) {
        return field;
    }

    if (mtq_firing && snapshot->magnetometer_uncal.meta.valid != 0U) {
        selected = &snapshot->magnetometer_uncal;
    } else if (snapshot->magnetometer_cal.meta.valid != 0U) {
        selected = &snapshot->magnetometer_cal;
    } else if (snapshot->magnetometer_uncal.meta.valid != 0U) {
        selected = &snapshot->magnetometer_uncal;
    }

    if (selected == NULL) {
        return field;
    }

    field.x = selected->value.x * microtesla_to_tesla;
    field.y = selected->value.y * microtesla_to_tesla;
    field.z = selected->value.z * microtesla_to_tesla;
    return field;
}

static vec3_t AppRuntime_SelectGyro(const imu_snapshot_t *snapshot)
{
    vec3_t gyro = {0.0f, 0.0f, 0.0f};
    if (snapshot == NULL || snapshot->gyro.meta.valid == 0U) {
        return gyro;
    }
    gyro.x = snapshot->gyro.value.x;
    gyro.y = snapshot->gyro.value.y;
    gyro.z = snapshot->gyro.value.z;
    return gyro;
}

static void app_sensor_console_step(void)
{
    imu_snapshot_t snapshot;
    power_snapshot_t power;
    uint32_t now_ms = HAL_GetTick();
    adcs_mode_t mode = OuterLoop_GetMode();

    IMU_ReadSnapshot(&snapshot);
    if (snapshot.publish_sequence == 0U) {
        log_printf_dma("IMU waiting snapshot=0 mode=%u imu_ready=%u", mode, g_imu_ready);
        return;
    }

    log_printf_dma(
        "IMU seq=%lu mode=%u gyro[v/e]=%u/%u (%.4f,%.4f,%.4f) mag[v/e]=%u/%u (%.2f,%.2f,%.2f) "
        "uncal[v/e]=%u/%u (%.2f,%.2f,%.2f) raw[v/e]=%u/%u (%d,%d,%d) rv[v/e]=%u/%u w=%.4f grv[v/e]=%u/%u w=%.4f "
        "age_ms[gyro=%lu mag=%lu raw=%lu]",
        (unsigned long)snapshot.publish_sequence,
        mode,
        snapshot.gyro.meta.valid, snapshot.gyro.meta.enabled,
        snapshot.gyro.value.x, snapshot.gyro.value.y, snapshot.gyro.value.z,
        snapshot.magnetometer_cal.meta.valid, snapshot.magnetometer_cal.meta.enabled,
        snapshot.magnetometer_cal.value.x, snapshot.magnetometer_cal.value.y, snapshot.magnetometer_cal.value.z,
        snapshot.magnetometer_uncal.meta.valid, snapshot.magnetometer_uncal.meta.enabled,
        snapshot.magnetometer_uncal.value.x, snapshot.magnetometer_uncal.value.y, snapshot.magnetometer_uncal.value.z,
        snapshot.magnetometer_raw.meta.valid, snapshot.magnetometer_raw.meta.enabled,
        snapshot.magnetometer_raw.value.x, snapshot.magnetometer_raw.value.y, snapshot.magnetometer_raw.value.z,
        snapshot.rotation_vector.meta.valid, snapshot.rotation_vector.meta.enabled, snapshot.rotation_vector.value.w,
        snapshot.game_rotation_vector.meta.valid, snapshot.game_rotation_vector.meta.enabled, snapshot.game_rotation_vector.value.w,
        (unsigned long)AppRuntime_ReportAgeMs(now_ms, &snapshot.gyro.meta),
        (unsigned long)AppRuntime_ReportAgeMs(now_ms, &snapshot.magnetometer_cal.meta),
        (unsigned long)AppRuntime_ReportAgeMs(now_ms, &snapshot.magnetometer_raw.meta));

    Power_ReadSnapshot(&power);
    log_printf_dma(
        "PWR seq=%lu batt_valid=0x%08lX I=%dmA Iavg=%dmA V=%umV Vavg=%umV temp_valid=0x%02X temp_cC=%d status=0x%02X age_ms=%lu",
        (unsigned long)power.publish_sequence,
        (unsigned long)power.battery.valid_mask,
        (int)power.battery.current_mA,
        (int)power.battery.average_current_mA,
        (unsigned int)power.battery.voltage_mV,
        (unsigned int)power.battery.average_voltage_mV,
        (unsigned int)power.digital_temp.valid_mask,
        (int)power.digital_temp.temperature_cC,
        (unsigned int)power.digital_temp.status,
        (unsigned long)power.source_age_ms);
}

void AppRuntime_OnControlTickFromISR(void)
{
    app_control_step();
}

static void PowerDataTask(void *argument)
{
    TickType_t next_wake = xTaskGetTickCount();
    (void)argument;
    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(POWER_SAMPLE_PERIOD_MS));
        app_sensor_step();
    }
}

static void ImuTask(void *argument)
{
    TickType_t next_wake = xTaskGetTickCount();
    (void)argument;
    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(IMU_SERVICE_PERIOD_MS));
        app_imu_step();
    }
}

static void TelemetryTask(void *argument)
{
    TickType_t next_wake = xTaskGetTickCount();
    uint32_t last_sensor_log_ms = 0U;
    (void)argument;
    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        app_telemetry_step();
        if ((uint32_t)(HAL_GetTick() - last_sensor_log_ms) >= SENSOR_LOG_PERIOD_MS) {
            app_sensor_console_step();
            last_sensor_log_ms = HAL_GetTick();
        }
    }
}

static void LoggerTask(void *argument)
{
    (void)argument;
    for (;;) {
        serial_log_process_tx();
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

static void ADCSTask(void *argument)
{
    static float dt_s = 0.01f;
    adcs_sensor_input_t adcs_in;
    adcs_output_t adcs_out;
    TickType_t next_wake = xTaskGetTickCount();
    (void)argument;

    for (;;) {
        imu_snapshot_t snapshot;
        imu_request_set_t control_request;
        mtq_state_t mtq_state;
        adcs_mode_t mode;
        bool mtq_firing = false;

        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(ADCS_PERIOD_MS));

        IMU_ReadSnapshot(&snapshot);
        (void)memset(&mtq_state, 0, sizeof(mtq_state));
        (void)InnerLoop_GetStateSnapshot(&mtq_state, 0U);
        mtq_firing = AppRuntime_IsMtqFiring(&mtq_state);
        mode = OuterLoop_GetMode();

        IMU_BuildControlRequestSet(mode, mtq_firing, &control_request);
        IMU_RequestSet(IMU_CLIENT_CONTROL, &control_request);

        adcs_in.mag_field = AppRuntime_SelectMagneticFieldTesla(&snapshot, mtq_firing);
        adcs_in.gyro = AppRuntime_SelectGyro(&snapshot);
        adcs_in.orientation = AppRuntime_SelectOrientation(&snapshot, mode, mtq_firing);

        OuterLoop_Update(&adcs_in, &adcs_out, dt_s);
        InnerLoop_SetTargetCurrent(adcs_out.dipole_request.x * MTQ_DIPOLE_TO_AMP,
                                   adcs_out.dipole_request.y * MTQ_DIPOLE_TO_AMP,
                                   adcs_out.dipole_request.z * MTQ_DIPOLE_TO_AMP);
    }
}

void AppRuntime_Init(void)
{
    AppRuntime_BootPrint("BOOT AppRuntime_Init enter");
    InnerLoop_Init();
    AppRuntime_BootPrint("BOOT InnerLoop_Init done");
    OuterLoop_Init();
    AppRuntime_BootPrint("BOOT OuterLoop_Init done");
    InnerLoop_SetTargetCurrent(0.0f, 0.0f, 0.0f);
    CurrentSensor_Init();
    CurrentSensor_SubmitSampleRequest();

    IMU_Init();
    Power_Init();
    IMU_RequestProfile(IMU_CLIENT_COMMS, IMU_PROFILE_TELEMETRY_BASIC);
    IMU_RequestProfile(IMU_CLIENT_CONTROL, IMU_PROFILE_CTRL_DETUMBLE);

    g_imu_ready = 0U;
    g_temp_sensor_ready = 0U;
    g_last_imu_init_attempt_ms = HAL_GetTick() - SENSOR_INIT_RETRY_MS;
    g_last_temp_init_attempt_ms = HAL_GetTick() - SENSOR_INIT_RETRY_MS;
    g_last_imu_reconcile_ms = 0U;
    g_next_reconcile_report = IMU_REPORT_ROTATION_VECTOR;
    g_power_avg_current_mA = 0.0f;
    g_power_avg_voltage_mV = 0.0f;
    g_power_avg_valid = 0U;

    log_printf_dma("AppRuntime init complete (deferred sensor init)");
    AppRuntime_BootPrint("BOOT AppRuntime_Init done");
}

void AppRuntime_Start(void)
{
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        AppRuntime_FatalStartup("HAL_TIM_Base_Start_IT(TIM6)");
    }
    if (xTaskCreate(ADCSTask, "ADCSTask", 1024U, NULL, 4U, &g_adcs_task_handle) != pdPASS) {
        AppRuntime_FatalStartup("xTaskCreate(ADCSTask)");
    }
    if (xTaskCreate(PowerDataTask, "PowerDataTask", 512U, NULL, 4U, NULL) != pdPASS) {
        AppRuntime_FatalStartup("xTaskCreate(PowerDataTask)");
    }
    if (xTaskCreate(ImuTask, "ImuTask", 896U, NULL, 4U, NULL) != pdPASS) {
        AppRuntime_FatalStartup("xTaskCreate(ImuTask)");
    }
    if (xTaskCreate(TelemetryTask, "TelemetryTask", 768U, NULL, 3U, NULL) != pdPASS) {
        AppRuntime_FatalStartup("xTaskCreate(TelemetryTask)");
    }
    if (xTaskCreate(LoggerTask, "LoggerTask", 512U, NULL, 2U, NULL) != pdPASS) {
        AppRuntime_FatalStartup("xTaskCreate(LoggerTask)");
    }

    vTaskStartScheduler();
    AppRuntime_FatalStartup("vTaskStartScheduler returned");
}

void AppRuntime_RunOnce(void)
{
    serial_log_process_tx();
}
