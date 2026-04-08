#include "app_runtime.h"

#include "main.h"
#include "config.h"
#include "inner_loop_control.h"
#include "manual_validation.h"
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
#include "queue.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define ADCS_PERIOD_MS 10U // 100Hz outer loop
#define TELEMETRY_PERIOD_MS 50U
#define CURRENT_SAMPLE_PERIOD_MS SENSOR_SAMPLE_PERIOD_MS
#define SENSOR_LOG_PERIOD_MS SENSOR_LOG_PRINT_PERIOD_MS
#define IMU_SERVICE_PERIOD_MS 2U
#define VALIDATION_PERIOD_MS 10U
#define SIM_PACKET_TIMEOUT_MS 20U
#define SENSOR_INIT_RETRY_MS 3000U
#define IMU_REENABLE_PERIOD_MS 1000U
#define IMU_STREAM_STALE_MS 250U
#define RAW_MAG_BASELINE_CAPTURE_MS 1500U
#define CONSOLE_RX_QUEUE_LEN 64U
#define CONSOLE_LINE_MAX_LEN 80U

TaskHandle_t g_adcs_task_handle = NULL; // Exported for UART ISR notification

extern bno085_t imu;

typedef struct
{
    float temp_c;
    uint8_t temp_valid;
    bno085_vec3_t mag_raw;
    uint8_t mag_raw_valid;
    uint32_t mag_raw_last_update_ms;
    uint32_t mag_raw_sample_count;
    bno085_vec3_t mag_raw_baseline;
    uint8_t mag_raw_baseline_valid;
    uint32_t mag_raw_baseline_first_ms;
    uint32_t mag_raw_baseline_sample_count;
    float mag_raw_baseline_sum_x;
    float mag_raw_baseline_sum_y;
    float mag_raw_baseline_sum_z;
} app_sensor_log_cache_t;

static ADT7420_Handle g_temp_sensor;
static uint8_t g_temp_sensor_ready = 0U;
static uint8_t g_imu_ready = 0U;
static uint32_t g_last_temp_init_attempt_ms = 0U;
static uint32_t g_last_imu_init_attempt_ms = 0U;
static uint32_t g_last_imu_reenable_ms = 0U;
static uint8_t g_validation_auto_started = 0U;
static app_sensor_log_cache_t g_sensor_log_cache = {0};
static manual_validation_state_t g_manual_validation;
static QueueHandle_t g_console_rx_queue = NULL;
static volatile uint32_t g_console_rx_drop_count = 0U;

static float AppValueOrZero(uint8_t valid, float value)
{
    return (valid != 0U) ? value : 0.0f;
}

static uint8_t AppImuStreamIsFresh(uint8_t valid, uint32_t last_update_ms, uint32_t now_ms)
{
    if (valid == 0U)
    {
        return 0U;
    }
    return ((uint32_t)(now_ms - last_update_ms) <= IMU_STREAM_STALE_MS) ? 1U : 0U;
}

static void AppSensorCache_PollImu(void)
{
    bno085_vec3_t mag_raw;
    uint32_t now_ms = HAL_GetTick();

    taskENTER_CRITICAL();
    if (BNO085_GetMagnetometerRaw(&imu, &mag_raw))
    {
        g_sensor_log_cache.mag_raw = mag_raw;
        g_sensor_log_cache.mag_raw_valid = 1U;
        g_sensor_log_cache.mag_raw_last_update_ms = now_ms;
        g_sensor_log_cache.mag_raw_sample_count++;
        if (g_sensor_log_cache.mag_raw_baseline_valid == 0U)
        {
            if (g_sensor_log_cache.mag_raw_baseline_sample_count == 0U)
            {
                g_sensor_log_cache.mag_raw_baseline_first_ms = now_ms;
            }
            if ((uint32_t)(now_ms - g_sensor_log_cache.mag_raw_baseline_first_ms) <= RAW_MAG_BASELINE_CAPTURE_MS)
            {
                g_sensor_log_cache.mag_raw_baseline_sum_x += mag_raw.x;
                g_sensor_log_cache.mag_raw_baseline_sum_y += mag_raw.y;
                g_sensor_log_cache.mag_raw_baseline_sum_z += mag_raw.z;
                g_sensor_log_cache.mag_raw_baseline_sample_count++;
            }
            if (g_sensor_log_cache.mag_raw_baseline_sample_count > 0U &&
                (uint32_t)(now_ms - g_sensor_log_cache.mag_raw_baseline_first_ms) >= RAW_MAG_BASELINE_CAPTURE_MS)
            {
                float inv_count = 1.0f / (float)g_sensor_log_cache.mag_raw_baseline_sample_count;
                g_sensor_log_cache.mag_raw_baseline.x = g_sensor_log_cache.mag_raw_baseline_sum_x * inv_count;
                g_sensor_log_cache.mag_raw_baseline.y = g_sensor_log_cache.mag_raw_baseline_sum_y * inv_count;
                g_sensor_log_cache.mag_raw_baseline.z = g_sensor_log_cache.mag_raw_baseline_sum_z * inv_count;
                g_sensor_log_cache.mag_raw_baseline.valid = true;
                g_sensor_log_cache.mag_raw_baseline_valid = 1U;
            }
        }
    }
    taskEXIT_CRITICAL();
}

static void AppSensorCache_SetTemperature(float temp_c)
{
    taskENTER_CRITICAL();
    g_sensor_log_cache.temp_c = temp_c;
    g_sensor_log_cache.temp_valid = 1U;
    taskEXIT_CRITICAL();
}

static void AppSensorCache_Snapshot(app_sensor_log_cache_t *out)
{
    if (out == NULL)
    {
        return;
    }
    taskENTER_CRITICAL();
    *out = g_sensor_log_cache;
    taskEXIT_CRITICAL();
}

static void AppManualValidation_Snapshot(manual_validation_state_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = g_manual_validation;
    taskEXIT_CRITICAL();
}

static void AppRuntime_BootPrint(const char *msg)
{
    if (msg == NULL)
    {
        return;
    }
    size_t n = strnlen(msg, 120U);
    if (n == 0U)
    {
        return;
    }
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)n, 20U);
    static uint8_t crlf[2] = {'\r', '\n'};
    (void)HAL_UART_Transmit(&huart2, crlf, 2U, 20U);
}

static void AppRuntime_ReenableMissingImuReports(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - g_last_imu_reenable_ms) < IMU_REENABLE_PERIOD_MS)
    {
        return;
    }

    uint8_t need_mag_raw = 0U;
    taskENTER_CRITICAL();
    need_mag_raw = (uint8_t)!AppImuStreamIsFresh(
        g_sensor_log_cache.mag_raw_valid,
        g_sensor_log_cache.mag_raw_last_update_ms,
        now_ms);
    taskEXIT_CRITICAL();

    if (need_mag_raw == 0U)
    {
        return;
    }

    g_last_imu_reenable_ms = now_ms;
    if (need_mag_raw != 0U)
    {
        (void)BNO085_EnableMagnetometer(&imu, BNO085_REPORT_INTERVAL_US);
        (void)BNO085_EnableRawMagnetometer(&imu, BNO085_REPORT_INTERVAL_US);
    }

}

static void AppRuntime_FatalStartup(const char *msg)
{
    if (msg != NULL)
    {
        char line[96];
        int n = snprintf(line, sizeof(line), "FATAL RTOS startup: %s", msg);
        if (n > 0)
        {
            AppRuntime_BootPrint(line);
        }
        log_printf_dma("FATAL RTOS startup: %s", msg);
        serial_log_process_tx();
    }
    Error_Handler();
}

/* ---------- Helper steps ---------- */

static void app_control_step(void)
{
    InnerLoop_Update();
}

static void app_sensor_step(void)
{
    uint32_t now_ms = HAL_GetTick();
    CurrentSensor_RunAsyncSample();
    CurrentSensor_SubmitSampleRequest();

    if (g_temp_sensor_ready == 0U)
    {
        if ((uint32_t)(now_ms - g_last_temp_init_attempt_ms) >= SENSOR_INIT_RETRY_MS)
        {
            g_last_temp_init_attempt_ms = now_ms;
            if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(ADT7420_I2C_ADDR_7BIT << 1), 2U, 20U) == HAL_OK &&
                ADT7420_Init(&g_temp_sensor, &hi2c2, ADT7420_I2C_ADDR_7BIT) == HAL_OK)
            {
                ADT7420_SubmitSampleRequest();
                g_temp_sensor_ready = 1U;
                log_printf_dma("ADT7420 init OK");
            }
        }
    }
    else
    {
        float temp_c = 0.0f;
        ADT7420_RunAsyncSample(&g_temp_sensor);
        ADT7420_SubmitSampleRequest();
        if (ADT7420_GetLatestSample(&temp_c, NULL, now_ms) != 0)
        {
            AppSensorCache_SetTemperature(temp_c);
        }
    }
}

static void app_telemetry_step(void)
{
#if HARDWARE_TELEPLOT_ENABLE
    mtq_state_t data;
    if (!InnerLoop_GetStateSnapshot(&data, 0U))
    {
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
#endif
}

static void app_manual_validation_step(void)
{
    manual_validation_state_t state;
    manual_validation_output_t output;

    AppManualValidation_Snapshot(&state);
    taskENTER_CRITICAL();
    if (g_validation_auto_started == 0U && g_sensor_log_cache.mag_raw_baseline_valid != 0U)
    {
        g_manual_validation.armed = 1U;
        g_validation_auto_started = 1U;
        state = g_manual_validation;
        log_printf_dma("Manual validation auto-armed after raw baseline capture");
    }
    taskEXIT_CRITICAL();
    ManualValidation_ComputeOutput(&state, HAL_GetTick(), &output);
    InnerLoop_SetTargetCurrent(output.target_current_x, output.target_current_y, output.target_current_z);
}

static void app_imu_step(void)
{
    uint32_t now_ms = HAL_GetTick();
    if (g_imu_ready == 0U)
    {
        if ((uint32_t)(now_ms - g_last_imu_init_attempt_ms) >= SENSOR_INIT_RETRY_MS)
        {
            g_last_imu_init_attempt_ms = now_ms;
            if (BNO085_Begin(&imu))
            {
                g_imu_ready = 1U;
                log_printf_dma("BNO085 init OK");
            }
            else
            {
                log_printf_dma("WARN BNO085 init retry failed");
            }
        }
    }
    for (uint32_t packets = 0U; packets < 16U; ++packets)
    {
        if (!BNO085_Service(&imu, NULL))
        {
            break;
        }
    }
    AppSensorCache_PollImu();

    if (g_imu_ready == 0U)
    {
        uint8_t has_stream = 0U;
        taskENTER_CRITICAL();
        if (AppImuStreamIsFresh(g_sensor_log_cache.mag_raw_valid, g_sensor_log_cache.mag_raw_last_update_ms, now_ms))
        {
            has_stream = 1U;
        }
        taskEXIT_CRITICAL();
        if (has_stream != 0U)
        {
            g_imu_ready = 1U;
            log_printf_dma("BNO085 stream active");
        }
    }
    AppRuntime_ReenableMissingImuReports(now_ms);
}

static void app_sensor_console_step(void)
{
    app_sensor_log_cache_t sensor;
    uint32_t now_ms = HAL_GetTick();
    uint8_t mag_raw_fresh;
    float delta_x;
    float delta_y;
    float delta_z;

    AppSensorCache_Snapshot(&sensor);
    mag_raw_fresh = AppImuStreamIsFresh(sensor.mag_raw_valid, sensor.mag_raw_last_update_ms, now_ms);

    if (mag_raw_fresh == 0U)
    {
        log_printf_dma(
            "IMU waiting mag_raw=%u age_ms=%lu baseline=%u baseline_samples=%lu imu_ready=%u",
            mag_raw_fresh,
            (unsigned long)(now_ms - sensor.mag_raw_last_update_ms),
            sensor.mag_raw_baseline_valid,
            (unsigned long)sensor.mag_raw_baseline_sample_count,
            g_imu_ready);
        return;
    }

    delta_x = sensor.mag_raw.x - sensor.mag_raw_baseline.x;
    delta_y = sensor.mag_raw.y - sensor.mag_raw_baseline.y;
    delta_z = sensor.mag_raw.z - sensor.mag_raw_baseline.z;

    log_printf_dma(
        "IMU RAW[adc]=(%.0f,%.0f,%.0f) RAW_BASELINE=(%.0f,%.0f,%.0f) RAW_DELTA=(%.0f,%.0f,%.0f) age_ms=%lu seq=%lu baseline=%u baseline_samples=%lu",
        AppValueOrZero(sensor.mag_raw_valid, sensor.mag_raw.x), AppValueOrZero(sensor.mag_raw_valid, sensor.mag_raw.y),
        AppValueOrZero(sensor.mag_raw_valid, sensor.mag_raw.z),
        AppValueOrZero(sensor.mag_raw_baseline_valid, sensor.mag_raw_baseline.x),
        AppValueOrZero(sensor.mag_raw_baseline_valid, sensor.mag_raw_baseline.y),
        AppValueOrZero(sensor.mag_raw_baseline_valid, sensor.mag_raw_baseline.z),
        sensor.mag_raw_baseline_valid ? delta_x : 0.0f,
        sensor.mag_raw_baseline_valid ? delta_y : 0.0f,
        sensor.mag_raw_baseline_valid ? delta_z : 0.0f,
        (unsigned long)(now_ms - sensor.mag_raw_last_update_ms),
        (unsigned long)sensor.mag_raw_sample_count,
        sensor.mag_raw_baseline_valid,
        (unsigned long)sensor.mag_raw_baseline_sample_count);
}

/* ---------- ISR entry point ---------- */

void AppRuntime_OnControlTickFromISR(void)
{
    app_control_step();
}

void AppRuntime_OnConsoleByteFromISR(uint8_t byte)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (g_console_rx_queue == NULL || xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return;
    }

    if (xQueueSendToBackFromISR(g_console_rx_queue, &byte, &higher_priority_task_woken) != pdPASS)
    {
        g_console_rx_drop_count++;
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/* ---------- FreeRTOS Tasks ---------- */

static void CurrentSensorTask(void *argument)
{
    (void)argument;
    TickType_t next_wake = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(CURRENT_SAMPLE_PERIOD_MS));
        app_sensor_step();
    }
}

static void ImuTask(void *argument)
{
    (void)argument;
    TickType_t next_wake = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(IMU_SERVICE_PERIOD_MS));
        app_imu_step();
    }
}

static void TelemetryTask(void *argument)
{
    (void)argument;
    TickType_t next_wake = xTaskGetTickCount();
    uint32_t last_sensor_log_ms = 0U;
    for (;;)
    {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        app_telemetry_step();
        uint32_t now_ms = HAL_GetTick();
        if ((uint32_t)(now_ms - last_sensor_log_ms) >= SENSOR_LOG_PERIOD_MS)
        {
            app_sensor_console_step();
            last_sensor_log_ms = now_ms;
        }
    }
}

static void ValidationTask(void *argument)
{
    (void)argument;
    TickType_t next_wake = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(VALIDATION_PERIOD_MS));
        app_manual_validation_step();
    }
}

static void ConsoleTask(void *argument)
{
    uint8_t byte = 0U;
    char line[CONSOLE_LINE_MAX_LEN];
    size_t len = 0U;

    (void)argument;

    for (;;)
    {
        if (xQueueReceive(g_console_rx_queue, &byte, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        if (byte == '\r')
        {
            continue;
        }
        if (byte == '\n')
        {
            manual_validation_state_t next_state;
            manual_validation_command_result_t result;
            char status[128];

            if (len == 0U)
            {
                continue;
            }

            line[len] = '\0';
            AppManualValidation_Snapshot(&next_state);
            result = ManualValidation_ApplyCommand(&next_state, line);
            if (result.code == MANUAL_VALIDATION_CMD_OK)
            {
                taskENTER_CRITICAL();
                g_manual_validation = next_state;
                taskEXIT_CRITICAL();
            }

            AppManualValidation_Snapshot(&next_state);
            ManualValidation_FormatStatus(&next_state, status, sizeof(status));
            log_printf_dma("CMD %s :: %s", result.message, status);
            len = 0U;
            continue;
        }

        if (len + 1U >= sizeof(line))
        {
            len = 0U;
            log_printf_dma("CMD error :: line too long");
            continue;
        }

        line[len++] = (char)byte;
    }
}

static void LoggerTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        serial_log_process_tx();
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

/* ---------- ADCS Task ---------- */

static float ClampF_RT(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

static void ADCSTask(void *argument)
{
    (void)argument;
    adcs_sensor_input_t adcs_in;
    adcs_output_t adcs_out;
    uint8_t logged_passive = 0U;

    TickType_t next_wake = xTaskGetTickCount();
    for (;;)
    {
        // In hardware mode, run at fixed 100Hz rate
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(ADCS_PERIOD_MS));

        // 1. Prepare ADCS Input
        // TODO: Read from physical sensors (BNO085)
        // For now, zero input (sensors handled by ImuTask)
        adcs_in.mag_field = (vec3_t){0.0f, 0.0f, 0.0f};
        adcs_in.gyro = (vec3_t){0.0f, 0.0f, 0.0f};
        adcs_in.orientation = (quat_t){1.0f, 0.0f, 0.0f, 0.0f};

        // 2. Run ADCS Algorithms
        OuterLoop_Update(&adcs_in, &adcs_out, 0.01f); // 100Hz
        (void)adcs_out;

        if (logged_passive == 0U)
        {
            log_printf_dma("ADCSTask passive: actuator owner=MANUAL_VALIDATION");
            logged_passive = 1U;
        }
    }
}

/* ---------- Init / Start / RunOnce ---------- */

void AppRuntime_Init(void)
{
    AppRuntime_BootPrint("BOOT AppRuntime_Init enter");
    if (g_console_rx_queue == NULL)
    {
        g_console_rx_queue = xQueueCreate(CONSOLE_RX_QUEUE_LEN, sizeof(uint8_t));
    }
    if (g_console_rx_queue == NULL)
    {
        AppRuntime_FatalStartup("xQueueCreate(console_rx)");
    }
    InnerLoop_Init();
    AppRuntime_BootPrint("BOOT InnerLoop_Init done");
    OuterLoop_Init();
    AppRuntime_BootPrint("BOOT OuterLoop_Init done");
    ManualValidation_Init(&g_manual_validation);
    InnerLoop_SetTargetCurrent(0.0f, 0.0f, 0.0f);
    CurrentSensor_SubmitSampleRequest();
    g_imu_ready = 0U;
    g_temp_sensor_ready = 0U;
    g_last_imu_init_attempt_ms = HAL_GetTick() - SENSOR_INIT_RETRY_MS;
    g_last_temp_init_attempt_ms = HAL_GetTick() - SENSOR_INIT_RETRY_MS;
    g_last_imu_reenable_ms = 0U;
    log_printf_dma("AppRuntime init complete (deferred sensor init)");
    log_printf_dma("Commands: val off|arm|disarm|axis x|y|z|mode step|square|pulse|current_ma N|period_ms N");
    AppRuntime_BootPrint("BOOT AppRuntime_Init done");
}

void AppRuntime_Start(void)
{
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        AppRuntime_FatalStartup("HAL_TIM_Base_Start_IT(TIM6)");
    }
    if (xTaskCreate(ADCSTask, "ADCSTask", 1024U, NULL, 4U, &g_adcs_task_handle) != pdPASS)
    {
        AppRuntime_FatalStartup("xTaskCreate(ADCSTask)");
    }
    if (xTaskCreate(CurrentSensorTask, "CurrentTask", 384U, NULL, 4U, NULL) != pdPASS)
    {
        AppRuntime_FatalStartup("xTaskCreate(CurrentTask)");
    }
    if (xTaskCreate(ImuTask, "ImuTask", 768U, NULL, 4U, NULL) != pdPASS)
    {
        AppRuntime_FatalStartup("xTaskCreate(ImuTask)");
    }
    if (xTaskCreate(ValidationTask, "ValidationTask", 512U, NULL, 4U, NULL) != pdPASS)
    {
        AppRuntime_FatalStartup("xTaskCreate(ValidationTask)");
    }
    if (xTaskCreate(TelemetryTask, "TelemetryTask", 512U, NULL, 3U, NULL) != pdPASS)
    {
        AppRuntime_FatalStartup("xTaskCreate(TelemetryTask)");
    }
    if (xTaskCreate(ConsoleTask, "ConsoleTask", 768U, NULL, 2U, NULL) != pdPASS)
    {
        AppRuntime_FatalStartup("xTaskCreate(ConsoleTask)");
    }
    if (xTaskCreate(LoggerTask, "LoggerTask", 512U, NULL, 2U, NULL) != pdPASS)
    {
        AppRuntime_FatalStartup("xTaskCreate(LoggerTask)");
    }
    vTaskStartScheduler();
    AppRuntime_FatalStartup("vTaskStartScheduler returned");
}

void AppRuntime_RunOnce(void)
{
    // After vTaskStartScheduler(), the RTOS runs all tasks.
    // This is only reached if scheduler hasn't started or returned (shouldn't happen).
    // Fallback: pump serial log.
    serial_log_process_tx();
}
