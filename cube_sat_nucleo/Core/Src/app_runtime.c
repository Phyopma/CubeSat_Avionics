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

#if !SIMULATION_MODE
typedef struct
{
    float temp_c;
    uint8_t temp_valid;
    bno085_quat_t grv;
    uint8_t grv_valid;
    bno085_quat_t quat;
    uint8_t quat_valid;
    bno085_vec3_t mag;
    uint8_t mag_valid;
    bno085_vec3_t gyro;
    uint8_t gyro_valid;
    bno085_vec3_t lin_vel;
    uint8_t lin_vel_valid;
} app_sensor_log_cache_t;

static ADT7420_Handle g_temp_sensor;
static uint8_t g_temp_sensor_ready = 0U;
static uint8_t g_imu_ready = 0U;
static uint32_t g_last_temp_init_attempt_ms = 0U;
static uint32_t g_last_imu_init_attempt_ms = 0U;
static uint32_t g_last_imu_reenable_ms = 0U;
static app_sensor_log_cache_t g_sensor_log_cache = {0};

static float AppValueOrZero(uint8_t valid, float value)
{
    return (valid != 0U) ? value : 0.0f;
}

static void AppSensorCache_PollImu(void)
{
    bno085_quat_t quat;
    bno085_quat_t grv;
    bno085_vec3_t mag;
    bno085_vec3_t gyro;
    bno085_vec3_t lin_vel;

    taskENTER_CRITICAL();
    if (BNO085_GetGameQuaternion(&imu, &grv))
    {
        g_sensor_log_cache.grv = grv;
        g_sensor_log_cache.grv_valid = 1U;
    }
    if (BNO085_GetQuaternion(&imu, &quat))
    {
        g_sensor_log_cache.quat = quat;
        g_sensor_log_cache.quat_valid = 1U;
    }
    if (BNO085_GetMagnetometer(&imu, &mag))
    {
        g_sensor_log_cache.mag = mag;
        g_sensor_log_cache.mag_valid = 1U;
    }
    if (BNO085_GetGyroscope(&imu, &gyro))
    {
        g_sensor_log_cache.gyro = gyro;
        g_sensor_log_cache.gyro_valid = 1U;
    }
    if (BNO085_GetLinearAcceleration(&imu, &lin_vel))
    {
        g_sensor_log_cache.lin_vel = lin_vel;
        g_sensor_log_cache.lin_vel_valid = 1U;
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

    uint8_t need_grv = 0U;
    uint8_t need_q = 0U;
    uint8_t need_mag = 0U;
    uint8_t need_gyro = 0U;
    uint8_t need_lin = 0U;
    taskENTER_CRITICAL();
    need_grv = (g_sensor_log_cache.grv_valid == 0U);
    need_q = (g_sensor_log_cache.quat_valid == 0U);
    need_mag = (g_sensor_log_cache.mag_valid == 0U);
    need_gyro = (g_sensor_log_cache.gyro_valid == 0U);
    need_lin = (g_sensor_log_cache.lin_vel_valid == 0U);
    taskEXIT_CRITICAL();

    if ((need_grv | need_q | need_mag | need_gyro | need_lin) == 0U)
    {
        return;
    }

    g_last_imu_reenable_ms = now_ms;
    if (need_q != 0U)
    {
        (void)BNO085_EnableRotationVector(&imu, BNO085_REPORT_INTERVAL_US);
    }
    if (need_grv != 0U)
    {
        (void)BNO085_EnableGameRotationVector(&imu, BNO085_REPORT_INTERVAL_US);
    }
    if (need_gyro != 0U)
    {
        (void)BNO085_EnableGyroscope(&imu, BNO085_REPORT_INTERVAL_US);
    }
    if (need_mag != 0U)
    {
        (void)BNO085_EnableMagnetometer(&imu, BNO085_REPORT_INTERVAL_US);
        (void)BNO085_EnableMagnetometerUncal(&imu, BNO085_REPORT_INTERVAL_US);
    }
    if (need_lin != 0U)
    {
        (void)BNO085_EnableLinearAccelerometer(&imu, BNO085_REPORT_INTERVAL_US);
        (void)BNO085_EnableAccelerometer(&imu, BNO085_REPORT_INTERVAL_US);
        (void)BNO085_EnableGravity(&imu, BNO085_REPORT_INTERVAL_US);
    }

    log_printf_dma(
        "BNO085 re-enable missing grv=%u q=%u mag=%u gyro=%u lin=%u",
        need_grv, need_q, need_mag, need_gyro, need_lin);
}
#endif

static void AppRuntime_FatalStartup(const char *msg)
{
    if (msg != NULL)
    {
        char line[96];
        int n = snprintf(line, sizeof(line), "FATAL RTOS startup: %s", msg);
        if (n > 0)
        {
#if !SIMULATION_MODE
            AppRuntime_BootPrint(line);
#endif
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

#if !SIMULATION_MODE
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
#else
    (void)now_ms;
#endif
}

static void app_telemetry_step(void)
{
#if !SIMULATION_MODE && HARDWARE_TELEPLOT_ENABLE
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

static void app_imu_step(void)
{
#if !SIMULATION_MODE
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
    (void)BNO085_Service(&imu, NULL);
    AppSensorCache_PollImu();

    if (g_imu_ready == 0U)
    {
        uint8_t has_stream = 0U;
        taskENTER_CRITICAL();
        if (g_sensor_log_cache.quat_valid || g_sensor_log_cache.grv_valid ||
            g_sensor_log_cache.mag_valid || g_sensor_log_cache.gyro_valid ||
            g_sensor_log_cache.lin_vel_valid)
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
#endif
}

static void app_sensor_console_step(void)
{
#if !SIMULATION_MODE
    app_sensor_log_cache_t sensor;
    AppSensorCache_Snapshot(&sensor);

    if (sensor.grv_valid == 0U || sensor.quat_valid == 0U ||
        sensor.mag_valid == 0U || sensor.gyro_valid == 0U || sensor.lin_vel_valid == 0U)
    {
        log_printf_dma(
            "SENS waiting valid temp=%u grv=%u q=%u mag=%u gyro=%u lin=%u imu_ready=%u",
            sensor.temp_valid, sensor.grv_valid, sensor.quat_valid,
            sensor.mag_valid, sensor.gyro_valid, sensor.lin_vel_valid,
            g_imu_ready);
        return;
    }

    log_printf_dma(
        "SENS temp=%.2fC GRV[wxyz]=(%.4f,%.4f,%.4f,%.4f) Q[wxyz]=(%.4f,%.4f,%.4f,%.4f) MAG[uT]=(%.2f,%.2f,%.2f) GYRO[rad/s]=(%.4f,%.4f,%.4f) LIN[m/s2]=(%.4f,%.4f,%.4f) valid[temp=%u]",
        AppValueOrZero(sensor.temp_valid, sensor.temp_c),
        AppValueOrZero(sensor.grv_valid, sensor.grv.real), AppValueOrZero(sensor.grv_valid, sensor.grv.i),
        AppValueOrZero(sensor.grv_valid, sensor.grv.j), AppValueOrZero(sensor.grv_valid, sensor.grv.k),
        AppValueOrZero(sensor.quat_valid, sensor.quat.real), AppValueOrZero(sensor.quat_valid, sensor.quat.i),
        AppValueOrZero(sensor.quat_valid, sensor.quat.j), AppValueOrZero(sensor.quat_valid, sensor.quat.k),
        AppValueOrZero(sensor.mag_valid, sensor.mag.x), AppValueOrZero(sensor.mag_valid, sensor.mag.y),
        AppValueOrZero(sensor.mag_valid, sensor.mag.z),
        AppValueOrZero(sensor.gyro_valid, sensor.gyro.x), AppValueOrZero(sensor.gyro_valid, sensor.gyro.y),
        AppValueOrZero(sensor.gyro_valid, sensor.gyro.z),
        AppValueOrZero(sensor.lin_vel_valid, sensor.lin_vel.x), AppValueOrZero(sensor.lin_vel_valid, sensor.lin_vel.y),
        AppValueOrZero(sensor.lin_vel_valid, sensor.lin_vel.z),
        sensor.temp_valid);
#endif
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
#if !SIMULATION_MODE
    uint32_t last_sensor_log_ms = 0U;
#endif
    for (;;)
    {
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        app_telemetry_step();
#if !SIMULATION_MODE
        uint32_t now_ms = HAL_GetTick();
        if ((uint32_t)(now_ms - last_sensor_log_ms) >= SENSOR_LOG_PERIOD_MS)
        {
            app_sensor_console_step();
            last_sensor_log_ms = now_ms;
        }
#endif
    }
}

static void LoggerTask(void *argument)
{
    (void)argument;
#if SIMULATION_MODE
    uint32_t last_wait_log_ms = 0U;
#endif
    for (;;)
    {
#if SIMULATION_MODE
        uint32_t now_ms = HAL_GetTick();
        uint32_t last_rx_ms = sim_last_packet_ms;
        if ((uint32_t)(now_ms - last_rx_ms) >= 1000U &&
            (uint32_t)(now_ms - last_wait_log_ms) >= 1000U)
        {
            log_printf_async("[SIM] waiting for host packets on UART2");
            last_wait_log_ms = now_ms;
        }
#endif
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
#if SIMULATION_MODE
    static float runtime_mtq_dipole_to_amp = MTQ_DIPOLE_TO_AMP;
    static float last_runtime_voltage = PIL_MAX_VOLTAGE;
    static float last_runtime_dipole = (1.0f / MTQ_DIPOLE_TO_AMP);
#endif

    adcs_sensor_input_t adcs_in;
    adcs_output_t adcs_out;

#if !SIMULATION_MODE
    TickType_t next_wake = xTaskGetTickCount();
#endif
    for (;;)
    {
#if SIMULATION_MODE
        // In sim mode, wait for packet notification but fail safe to zero command
        // if packet flow stalls (prevents stale torque from being held indefinitely).
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SIM_PACKET_TIMEOUT_MS)) == 0U)
        {
            InnerLoop_SetTargetCurrent(0.0f, 0.0f, 0.0f);
            continue;
        }
#else
        // In hardware mode, run at fixed 100Hz rate
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(ADCS_PERIOD_MS));
#endif

        // 1. Prepare ADCS Input
#if SIMULATION_MODE
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
#else
        // TODO: Read from physical sensors (BNO085)
        // For now, zero input (sensors handled by ImuTask)
        adcs_in.mag_field = (vec3_t){0.0f, 0.0f, 0.0f};
        adcs_in.gyro = (vec3_t){0.0f, 0.0f, 0.0f};
        adcs_in.orientation = (quat_t){1.0f, 0.0f, 0.0f, 0.0f};
#endif

        // 2. Run ADCS Algorithms
#if SIMULATION_MODE
        OuterLoop_Update(&adcs_in, &adcs_out, sim_input.dt);
#else
        OuterLoop_Update(&adcs_in, &adcs_out, 0.01f); // 100Hz
#endif

        // 3. Command the Inner Loop
        float mtq_dipole_to_amp = MTQ_DIPOLE_TO_AMP;
#if SIMULATION_MODE
        mtq_dipole_to_amp = runtime_mtq_dipole_to_amp;
#endif
        float target_x = adcs_out.dipole_request.x * mtq_dipole_to_amp;
        float target_y = adcs_out.dipole_request.y * mtq_dipole_to_amp;
        float target_z = adcs_out.dipole_request.z * mtq_dipole_to_amp;

        InnerLoop_SetTargetCurrent(target_x, target_y, target_z);
    }
}

/* ---------- Init / Start / RunOnce ---------- */

void AppRuntime_Init(void)
{
#if !SIMULATION_MODE
    AppRuntime_BootPrint("BOOT AppRuntime_Init enter");
#endif
    InnerLoop_Init();
#if !SIMULATION_MODE
    AppRuntime_BootPrint("BOOT InnerLoop_Init done");
#endif
    OuterLoop_Init();
#if !SIMULATION_MODE
    AppRuntime_BootPrint("BOOT OuterLoop_Init done");
#endif
    InnerLoop_SetTargetCurrent(0.0f, 0.0f, 0.0f);
    CurrentSensor_SubmitSampleRequest();
#if !SIMULATION_MODE
    g_imu_ready = 0U;
    g_temp_sensor_ready = 0U;
    g_last_imu_init_attempt_ms = HAL_GetTick() - SENSOR_INIT_RETRY_MS;
    g_last_temp_init_attempt_ms = HAL_GetTick() - SENSOR_INIT_RETRY_MS;
    g_last_imu_reenable_ms = 0U;
    log_printf_dma("AppRuntime init complete (deferred sensor init)");
    AppRuntime_BootPrint("BOOT AppRuntime_Init done");
#endif
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
#if !SIMULATION_MODE
    if (xTaskCreate(LoggerTask, "LoggerTask", 512U, NULL, 2U, NULL) != pdPASS)
    {
        AppRuntime_FatalStartup("xTaskCreate(LoggerTask)");
    }
#endif
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
