#include "app_runtime.h"

#include "main.h"
#include "inner_loop_control.h"
#include "current_sensor.h"
#include "imu_bno085.h"
#include "teleplot.h"
#include "serial_log_dma.h"

#if defined(__has_include)
#if __has_include("FreeRTOS.h")
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#define APP_HAS_FREERTOS 1
#else
#define APP_HAS_FREERTOS 0
#endif
#else
#define APP_HAS_FREERTOS 0
#endif

#define CTRL_PERIOD_MS 1U
#define TELEMETRY_PERIOD_MS 50U
#define CURRENT_SAMPLE_PERIOD_MS 1U
#define IMU_SERVICE_PERIOD_MS 2U

#if APP_HAS_FREERTOS
static TaskHandle_t g_control_task_handle = NULL;
#endif

extern bno085_t imu;

static volatile uint8_t g_control_tick_pending = 0U;

static void app_control_step(void)
{
    InnerLoop_Update();
}

static void app_sensor_step(void)
{
    CurrentSensor_RunAsyncSample();
}

static void app_telemetry_step(void)
{
#ifndef SIMULATION_MODE
    mtq_state_t data;
    if (!InnerLoop_GetStateSnapshot(&data, 0U)) {
        return;
    }
    Teleplot_Update("Target", data.target_current * 1000.0f);
    Teleplot_Update("Current", data.measured_current * 1000.0f);
    Teleplot_Update("Error", (data.target_current - data.measured_current) * 1000.0f);
    Teleplot_Update("Voltage", data.command_voltage);
#endif
}

static void app_imu_step(void)
{
#if !defined(SIMULATION_MODE)
    (void)BNO085_Service(&imu, NULL);
#endif
}

void AppRuntime_OnControlTickFromISR(void)
{
#if APP_HAS_FREERTOS
    if (g_control_task_handle != NULL) {
        BaseType_t hpw = pdFALSE;
        vTaskNotifyGiveFromISR(g_control_task_handle, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
#else
    g_control_tick_pending = 1U;
#endif
}

#if APP_HAS_FREERTOS
static void ControlTask(void *argument)
{
    (void)argument;
    TickType_t next_wake = xTaskGetTickCount();
    for (;;) {
        ulTaskNotifyTake(pdTRUE, 0U);
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(CTRL_PERIOD_MS));
        app_control_step();
    }
}

static void CurrentSensorTask(void *argument)
{
    (void)argument;
    TickType_t next_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(CURRENT_SAMPLE_PERIOD_MS));
        app_sensor_step();
    }
}

static void ImuTask(void *argument)
{
    (void)argument;
    TickType_t next_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(IMU_SERVICE_PERIOD_MS));
        app_imu_step();
    }
}

static void TelemetryTask(void *argument)
{
    (void)argument;
    TickType_t next_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        app_telemetry_step();
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
#endif

void AppRuntime_Init(void)
{
    InnerLoop_Init();
    InnerLoop_SetTargetCurrentAsync(0.03f, 0U);
    CurrentSensor_SubmitSampleRequest();
}

void AppRuntime_Start(void)
{
#if APP_HAS_FREERTOS
    xTaskCreate(ControlTask, "ControlTask", 512U, NULL, 5U, &g_control_task_handle);
    xTaskCreate(CurrentSensorTask, "CurrentTask", 384U, NULL, 4U, NULL);
    xTaskCreate(ImuTask, "ImuTask", 768U, NULL, 4U, NULL);
    xTaskCreate(TelemetryTask, "TelemetryTask", 512U, NULL, 3U, NULL);
    xTaskCreate(LoggerTask, "LoggerTask", 512U, NULL, 2U, NULL);
    vTaskStartScheduler();
#endif
}

void AppRuntime_RunOnce(void)
{
#if APP_HAS_FREERTOS
    serial_log_process_tx();
#else
    static uint32_t next_sensor_ms = 0U;
    static uint32_t next_imu_ms = 0U;
    static uint32_t next_telemetry_ms = 0U;
    uint32_t now = HAL_GetTick();

    if (g_control_tick_pending != 0U) {
        g_control_tick_pending = 0U;
        app_control_step();
    }

    if ((int32_t)(now - next_sensor_ms) >= 0) {
        next_sensor_ms = now + CURRENT_SAMPLE_PERIOD_MS;
        app_sensor_step();
    }

    if ((int32_t)(now - next_imu_ms) >= 0) {
        next_imu_ms = now + IMU_SERVICE_PERIOD_MS;
        app_imu_step();
    }

    if ((int32_t)(now - next_telemetry_ms) >= 0) {
        next_telemetry_ms = now + TELEMETRY_PERIOD_MS;
        app_telemetry_step();
    }

    serial_log_process_tx();
#endif
}
