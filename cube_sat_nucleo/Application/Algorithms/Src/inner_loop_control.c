#include "inner_loop_control.h"
#include "config.h"
#include "main.h"
#include "hbridge.h"
#include "current_sensor.h"
#include "pi_controller.h"
#include "stdio.h"
#include "math.h"
#include "outer_loop_control.h" // Required for OuterLoop_GetMode

// Private State Variables
PI_Config_t pi_x, pi_y, pi_z;
mtq_state_t state;

#ifndef SIMULATION_MODE
static float filtered_current_z = 0.0f; // Only Z has real sensor for now
#endif

// HITL Sync Flag
volatile uint8_t sim_data_ready = 0;

void InnerLoop_Update_SimDataAvailable(void) {
    sim_data_ready = 1;
}

void InnerLoop_Init(void)
{
    // 1. Initialize Drivers
    HBridge_Init();
    CurrentSensor_Init();

    // 2. Initialize PI Controllers (Same gains for all axes for now)
    // Parameters moved to Application/Algorithms/Inc/config.h
    PI_Init(&pi_x, PIL_KP, PIL_KI, PIL_T, PIL_MAX_VOLTAGE);
    PI_Init(&pi_y, PIL_KP, PIL_KI, PIL_T, PIL_MAX_VOLTAGE);
    PI_Init(&pi_z, PIL_KP, PIL_KI, PIL_T, PIL_MAX_VOLTAGE);
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
    // === MODE A: OPEN LOOP (Active) ===
    // Physics Model: V = I * R
    state.command_voltage_x = fmaxf(-MAX_OUTPUT_VOLTAGE, fminf(MAX_OUTPUT_VOLTAGE, state.target_current_x * MTQ_COIL_RESISTANCE));
    state.command_voltage_y = fmaxf(-MAX_OUTPUT_VOLTAGE, fminf(MAX_OUTPUT_VOLTAGE, state.target_current_y * MTQ_COIL_RESISTANCE));
    state.command_voltage_z = fmaxf(-MAX_OUTPUT_VOLTAGE, fminf(MAX_OUTPUT_VOLTAGE, state.target_current_z * MTQ_COIL_RESISTANCE));

#else
#ifdef SIMULATION_MODE
    // === MODE B-Sim: HITL SIMULATION ===
    // 1. In Simulation Mode, we override the PHYSICAL Sensor readings
    //    with data received from Python via UART.
    state.measured_current_x = sim_input.current_amps_x;
    state.measured_current_y = sim_input.current_amps_y;
    state.measured_current_z = sim_input.current_amps_z;
    
    // 2. Run Control Logic (PI or Open Loop) when new data arrives
    if (sim_data_ready) {
        if (sim_input.debug_flags & 0x01) {
            // === OPEN LOOP OVERRIDE (Debug) ===
            // Force V = I * R (Bypassing PI)
            state.command_voltage_x = fmaxf(-MAX_OUTPUT_VOLTAGE, fminf(MAX_OUTPUT_VOLTAGE, state.target_current_x * MTQ_COIL_RESISTANCE));
            state.command_voltage_y = fmaxf(-MAX_OUTPUT_VOLTAGE, fminf(MAX_OUTPUT_VOLTAGE, state.target_current_y * MTQ_COIL_RESISTANCE));
            state.command_voltage_z = fmaxf(-MAX_OUTPUT_VOLTAGE, fminf(MAX_OUTPUT_VOLTAGE, state.target_current_z * MTQ_COIL_RESISTANCE));
        } else {
            // === CLOSED LOOP (PI) ===
            state.command_voltage_x = PI_Update(&pi_x, state.target_current_x, state.measured_current_x);
            state.command_voltage_y = PI_Update(&pi_y, state.target_current_y, state.measured_current_y);
            state.command_voltage_z = PI_Update(&pi_z, state.target_current_z, state.measured_current_z);
        }
        sim_data_ready = 0;
    }
    
    // 3. Send Output to Simulator (Rate Limited to 100Hz to avoid UART saturation)
    static uint32_t last_telemetry_tick = 0;
    uint32_t current_tick = HAL_GetTick();
    if (current_tick - last_telemetry_tick >= 10) { // 10ms = 100Hz
        sim_output.header = 0x62B5; // Sync Word (Sends 0xB5 then 0x62 on Little Endian)
        sim_output.command_voltage_x = state.command_voltage_x;
        sim_output.command_voltage_y = state.command_voltage_y;
        sim_output.command_voltage_z = state.command_voltage_z;
        sim_output.adcs_mode = (uint8_t)OuterLoop_GetMode();
        
        HAL_UART_Transmit_IT(&huart2, (uint8_t*)&sim_output, sizeof(sim_output));
        last_telemetry_tick = current_tick;
    }
    
#else
    // === MODE C: REAL HARDWARE (Partial) ===
    // X and Y have no sensors, assume 0 measurement (Open Loop-ish for PI)
    // Or just set measured = 0.
    state.measured_current_x = 0.0f;
    state.measured_current_y = 0.0f;

    // Z-Axis: Read Raw Hardware Value
    float raw_val_z = CurrentSensor_Read_Amps();

    // Inject Sign (Polarity logic for Z)
    float signed_raw_z = (state.command_voltage_z < 0.0f) ? -1.0f * fabsf(raw_val_z) : fabsf(raw_val_z);

    // Low Pass Filter for Z
    filtered_current_z = (filtered_current_z * 0.90f) + (signed_raw_z * 0.10f);
    state.measured_current_z = filtered_current_z;

    // Run PI on all axes (1kHz is fine for real hardware)
    state.command_voltage_x = PI_Update(&pi_x, state.target_current_x, state.measured_current_x);
    state.command_voltage_y = PI_Update(&pi_y, state.target_current_y, state.measured_current_y);
    state.command_voltage_z = PI_Update(&pi_z, state.target_current_z, state.measured_current_z);
#endif
#endif

    // 3. Apply Command to H-Bridge Drivers
    // Axis 0 = X, 1 = Y, 2 = Z
    // This MUST run every 1ms to keep PWM active/updated
    HBridge_SetVoltage(0, state.command_voltage_x, MAX_OUTPUT_VOLTAGE);
    HBridge_SetVoltage(1, state.command_voltage_y, MAX_OUTPUT_VOLTAGE);
    HBridge_SetVoltage(2, state.command_voltage_z, MAX_OUTPUT_VOLTAGE);
}

mtq_state_t InnerLoop_GetState(void)
{
    return state;
}
