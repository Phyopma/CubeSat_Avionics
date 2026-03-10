#include "app_runtime.h"

#include "main.h"
#include "config.h"
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
#include "queue.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define ADCS_PERIOD_MS 10U // 100Hz outer loop
#define TELEMETRY_PERIOD_MS 50U
#define CURRENT_SAMPLE_PERIOD_MS SENSOR_SAMPLE_PERIOD_MS
#define SENSOR_LOG_PERIOD_MS SENSOR_LOG_PRINT_PERIOD_MS
#define IMU_SERVICE_PERIOD_MS 2U
#define SIM_PACKET_TIMEOUT_MS 20U
#define SENSOR_INIT_RETRY_MS 3000U
#define IMU_REENABLE_PERIOD_MS 1000U

TaskHandle_t g_adcs_task_handle = NULL; // Exported for UART ISR notification

extern bno085_t imu;


static void AppRuntime_FatalStartup(const char *msg)
{
    if (msg != NULL)
    {
        char line[96];
        int n = snprintf(line, sizeof(line), "FATAL RTOS startup: %s", msg);
        if (n > 0)
        {
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

    (void)now_ms;
}

static void app_telemetry_step(void)
{
}

static void app_imu_step(void)
{
}

static void app_sensor_console_step(void)
{
}

/* ---------- ISR entry point ---------- */

void AppRuntime_OnControlTickFromISR(void)
{
    app_control_step();
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
    for (;;)
    {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        app_telemetry_step();
    }
}

static void LoggerTask(void *argument)
{
    (void)argument;
    uint32_t last_wait_log_ms = 0U;
    for (;;)
    {
        uint32_t now_ms = HAL_GetTick();
        uint32_t last_rx_ms = sim_last_packet_ms;
        if ((uint32_t)(now_ms - last_rx_ms) >= 1000U &&
            (uint32_t)(now_ms - last_wait_log_ms) >= 1000U)
        {
            log_printf_async("[SIM] waiting for host packets on UART2");
            last_wait_log_ms = now_ms;
        }
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

    // Runtime state for dynamic gain tracking
    static float last_kbdot = -1.0f, last_kp = -1.0f, last_ki = -1.0f, last_kd = -1.0f;
    static float last_c_damp = -1.0f, last_i_virtual = -1.0f;
    static uint8_t last_force_mode = 0xFF;
    static uint8_t last_reset_request = 0;
    static float runtime_mtq_dipole_to_amp = MTQ_DIPOLE_TO_AMP;
    static float last_runtime_voltage = PIL_MAX_VOLTAGE;
    static float last_runtime_dipole = (1.0f / MTQ_DIPOLE_TO_AMP);

    adcs_sensor_input_t adcs_in;
    adcs_output_t adcs_out;

    for (;;)
    {
        // In sim mode, wait for packet notification but fail safe to zero command
        // if packet flow stalls (prevents stale torque from being held indefinitely).
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SIM_PACKET_TIMEOUT_MS)) == 0U)
        {
            InnerLoop_SetTargetCurrent(0.0f, 0.0f, 0.0f);
            continue;
        }

        // 1. Prepare ADCS Input
        adcs_in.mag_field = (vec3_t){sim_input.mag_x, sim_input.mag_y, sim_input.mag_z};
        adcs_in.gyro = (vec3_t){sim_input.gyro_x, sim_input.gyro_y, sim_input.gyro_z};
        adcs_in.orientation = (quat_t){sim_input.q_w, sim_input.q_x, sim_input.q_y, sim_input.q_z};

        // Dynamic mode forcing
        uint8_t force_mode = (sim_input.debug_flags >> 1) & 0x03;
        uint8_t reset_request = (sim_input.debug_flags >> 3) & 0x01;
        if (force_mode != last_force_mode)
        {
            OuterLoop_SetForcedMode(force_mode);
            last_force_mode = force_mode;
        }
        if (reset_request && !last_reset_request)
        {
            OuterLoop_ResetControllerState();
        }
        last_reset_request = reset_request;

        // Dynamic voltage/dipole overrides
        float runtime_voltage = ClampF_RT(0.001f * (float)sim_input.max_voltage_mV, 0.5f, 12.0f);
        float runtime_dipole = ClampF_RT(0.001f * (float)sim_input.dipole_strength_milli, 0.1f, 20.0f);

        if (fabsf(runtime_voltage - last_runtime_voltage) > 1e-4f)
        {
            InnerLoop_SetVoltageLimit(runtime_voltage);
            last_runtime_voltage = runtime_voltage;
        }
        if (fabsf(runtime_dipole - last_runtime_dipole) > 1e-6f)
        {
            runtime_mtq_dipole_to_amp = 1.0f / runtime_dipole;
            last_runtime_dipole = runtime_dipole;
        }

        // Dynamic gains from sim
        if (force_mode == 2)
        {
            // Forced SPIN mode: kp/kd fields carry Kane parameters
            if (sim_input.kp != last_c_damp || sim_input.kd != last_i_virtual)
            {
                OuterLoop_SetSpinParams(sim_input.kp, sim_input.kd);
                last_c_damp = sim_input.kp;
                last_i_virtual = sim_input.kd;
            }
            last_kbdot = sim_input.k_bdot;
            last_kp = sim_input.kp;
            last_ki = sim_input.ki;
            last_kd = sim_input.kd;
        }
        else if (sim_input.k_bdot != last_kbdot || sim_input.kp != last_kp ||
                 sim_input.ki != last_ki || sim_input.kd != last_kd)
        {
            OuterLoop_SetGains(sim_input.k_bdot, sim_input.kp, sim_input.ki, sim_input.kd);
            last_kbdot = sim_input.k_bdot;
            last_kp = sim_input.kp;
            last_ki = sim_input.ki;
            last_kd = sim_input.kd;
        }

        // 2. Run ADCS Algorithms
        OuterLoop_Update(&adcs_in, &adcs_out, sim_input.dt);

        // 3. Command the Inner Loop
        float mtq_dipole_to_amp = MTQ_DIPOLE_TO_AMP;
        mtq_dipole_to_amp = runtime_mtq_dipole_to_amp;
        float target_x = adcs_out.dipole_request.x * mtq_dipole_to_amp;
        float target_y = adcs_out.dipole_request.y * mtq_dipole_to_amp;
        float target_z = adcs_out.dipole_request.z * mtq_dipole_to_amp;

        InnerLoop_SetTargetCurrent(target_x, target_y, target_z);
    }
}

/* ---------- Init / Start / RunOnce ---------- */

void AppRuntime_Init(void)
{
    InnerLoop_Init();
    OuterLoop_Init();
    InnerLoop_SetTargetCurrent(0.0f, 0.0f, 0.0f);
    CurrentSensor_SubmitSampleRequest();
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
    if (xTaskCreate(TelemetryTask, "TelemetryTask", 512U, NULL, 3U, NULL) != pdPASS)
    {
        AppRuntime_FatalStartup("xTaskCreate(TelemetryTask)");
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
