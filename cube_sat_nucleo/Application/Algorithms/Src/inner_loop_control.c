/* Application/Algorithms/Src/inner_loop_control.c */
#include "inner_loop_control.h"
#include "config.h"
#include "main.h"
#include "hbridge.h"
#include "current_sensor.h"
#include "pi_controller.h"
#include "outer_loop_control.h"
#include "stdio.h"
#include "math.h"

// Private State Variables
PI_Config_t pi_x, pi_y, pi_z;
mtq_state_t state;
static float runtime_voltage_limit = PIL_MAX_VOLTAGE;
static volatile uint8_t g_state_valid = 0U;

static float filtered_current_z = 0.0f; // Only Z has real sensor for now

// HITL Sync Flag
volatile uint8_t sim_data_ready = 0;

void InnerLoop_Update_SimDataAvailable(void)
{
    sim_data_ready = 1;
}

static float ClampF(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

static int16_t QuantizeQ15(float value, float full_scale, uint8_t *saturated)
{
    if (full_scale <= 1e-12f)
    {
        if (saturated != NULL)
        {
            *saturated = 1u;
        }
        return 0;
    }

    float normalized = value / full_scale;
    if (normalized > 1.0f)
    {
        normalized = 1.0f;
        if (saturated != NULL)
        {
            *saturated = 1u;
        }
    }
    else if (normalized < -1.0f)
    {
        normalized = -1.0f;
        if (saturated != NULL)
        {
            *saturated = 1u;
        }
    }

    float scaled = normalized * 32767.0f;
    if (scaled >= 32767.0f)
    {
        return 32767;
    }
    if (scaled <= -32768.0f)
    {
        return -32768;
    }
    return (int16_t)lroundf(scaled);
}

void InnerLoop_SetVoltageLimit(float v_limit)
{
    float clamped = ClampF(v_limit, 0.5f, 12.0f);
    runtime_voltage_limit = clamped;

    pi_x.limMax = clamped;
    pi_x.limMin = -clamped;
    pi_y.limMax = clamped;
    pi_y.limMin = -clamped;
    pi_z.limMax = clamped;
    pi_z.limMin = -clamped;

    pi_x.integrator = ClampF(pi_x.integrator, pi_x.limMin, pi_x.limMax);
    pi_y.integrator = ClampF(pi_y.integrator, pi_y.limMin, pi_y.limMax);
    pi_z.integrator = ClampF(pi_z.integrator, pi_z.limMin, pi_z.limMax);
}

float InnerLoop_GetVoltageLimit(void)
{
    return runtime_voltage_limit;
}

void InnerLoop_Init(void)
{
    // 1. Initialize Drivers
    HBridge_Init();
    CurrentSensor_Init();

    // 2. Initialize PI Controllers (Same gains for all axes)
    // Parameters from Application/Algorithms/Inc/config.h
    PI_Init(&pi_x, PIL_KP, PIL_KI, PIL_T, PIL_MAX_VOLTAGE);
    PI_Init(&pi_y, PIL_KP, PIL_KI, PIL_T, PIL_MAX_VOLTAGE);
    PI_Init(&pi_z, PIL_KP, PIL_KI, PIL_T, PIL_MAX_VOLTAGE);
    InnerLoop_SetVoltageLimit(PIL_MAX_VOLTAGE);
}

void InnerLoop_SetTargetCurrent(float x, float y, float z)
{
    state.target_current_x = x;
    state.target_current_y = y;
    state.target_current_z = z;
}

// This function runs inside the TIM6 Interrupt (1kHz)
void InnerLoop_Update(void)
{
#ifdef MTQ_MODE_OPEN_LOOP
    // === MODE A: OPEN LOOP ===
    state.command_voltage_x = fmaxf(-runtime_voltage_limit, fminf(runtime_voltage_limit, state.target_current_x * MTQ_COIL_RESISTANCE));
    state.command_voltage_y = fmaxf(-runtime_voltage_limit, fminf(runtime_voltage_limit, state.target_current_y * MTQ_COIL_RESISTANCE));
    state.command_voltage_z = fmaxf(-runtime_voltage_limit, fminf(runtime_voltage_limit, state.target_current_z * MTQ_COIL_RESISTANCE));

#else
    // === MODE C: REAL HARDWARE (Partial) ===
    // X and Y have no sensors, assume 0 measurement
    state.measured_current_x = 0.0f;
    state.measured_current_y = 0.0f;

    // Z-axis current sample is produced asynchronously in task context.
    // Do not touch I2C from TIM6 ISR path.
    float sampled_current_z = 0.0f;
    float raw_val_z = fabsf(filtered_current_z);
    if (CurrentSensor_GetLatestSample(&sampled_current_z, NULL, HAL_GetTick()) != 0)
    {
        raw_val_z = fabsf(sampled_current_z);
    }

    // Inject Sign (Polarity logic for Z)
    float signed_raw_z = (state.command_voltage_z < 0.0f) ? -1.0f * fabsf(raw_val_z) : fabsf(raw_val_z);

    // Low Pass Filter for Z
    filtered_current_z = (filtered_current_z * 0.90f) + (signed_raw_z * 0.10f);
    state.measured_current_z = filtered_current_z;

    // Run PI on all axes
    state.command_voltage_x = PI_Update(&pi_x, state.target_current_x, state.measured_current_x);
    state.command_voltage_y = PI_Update(&pi_y, state.target_current_y, state.measured_current_y);
    state.command_voltage_z = PI_Update(&pi_z, state.target_current_z, state.measured_current_z);
#endif

    // 3. Apply Command to H-Bridge Drivers (Axis 0=X, 1=Y, 2=Z)
    HBridge_SetVoltage(0, state.command_voltage_x, runtime_voltage_limit);
    HBridge_SetVoltage(1, state.command_voltage_y, runtime_voltage_limit);
    HBridge_SetVoltage(2, state.command_voltage_z, runtime_voltage_limit);
    g_state_valid = 1U;
}

mtq_state_t InnerLoop_GetState(void)
{
    return state;
}

int InnerLoop_GetStateSnapshot(mtq_state_t *out, uint32_t timeout_ms)
{
    (void)timeout_ms;
    if (out == NULL || g_state_valid == 0U)
    {
        return 0;
    }
    *out = state;
    return 1;
}
