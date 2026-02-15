/* Application/Algorithms/Src/inner_loop_control.c */
#include "inner_loop_control.h"
#include "main.h"
#include "hbridge.h"
#include "current_sensor.h"
#include "pi_controller.h"
#include "stdio.h"
#include "math.h"

// Private State Variables
PI_Config_t pi_ctrl;
mtq_state_t state;
static float filtered_current = 0.0f;
static volatile uint8_t g_state_valid = 0U;

//void InnerLoop_PrintTelemetry(char *buffer)
//{
//    // Format: "Target,Measured,Voltage\r\n"
//    // Multiplied by 1000 to print as integers (mA) avoids float %f issues
//    sprintf(buffer, "%d,%d,%d\r\n",
//            (int)(target_current * 1000),
//            (int)(current_measure * 1000), // Will be 0 in Open Loop
//            (int)(cmd_voltage * 1000));
//}
// tuning the PI controller

void InnerLoop_Init(void)
{
    // 1. Initialize Drivers
    HBridge_Init();
    CurrentSensor_Init();

    // 2. Initialize PI Controller
	// Kp = 5.0 (Restored original fine-tuned value)
	// Ki = 100.0 (Fast correction of steady state)
	// T = 0.001 (1kHz loop)
	// Limit = 5.0V (Max voltage we can output)
	PI_Init(&pi_ctrl, 5.0f, 1500.0f, 0.001f, MAX_OUTPUT_VOLTAGE);
}

void InnerLoop_SetTargetCurrent(float current_amps)
{
    state.target_current = current_amps;

}

int InnerLoop_SetTargetCurrentAsync(float current_amps, uint32_t timeout_ms)
{
    (void)timeout_ms;
    InnerLoop_SetTargetCurrent(current_amps);
    return 1;
}

// This function runs inside the TIM6 Interrupt (1kHz)
void InnerLoop_Update(void)
{
#ifdef MTQ_MODE_OPEN_LOOP
    // === MODE A: OPEN LOOP (Active) ===
    // Physics Model: V = I * R
    // This ignores the sensor and assumes resistance is constant.
	state.command_voltage = state.target_current * MTQ_COIL_RESISTANCE;

#else
#ifdef SIMULATION_MODE
    // === MODE B-Sim: HITL SIMULATION ===
	// 1. In Simulation Mode, we override the PHYSICAL Sensor readings
	//    with data received from Python via UART.
    state.measured_current = sim_input.current_amps;
    
    // 2. We also obey the TARGET requested by Python (since Outer Loop isn't running)
    state.target_current = sim_input.target_current_cmd;
    
    // 2. Run PI
    state.command_voltage = PI_Update(&pi_ctrl, state.target_current, state.measured_current);
    
    // 3. Send Output to Simulator
    sim_output.header = 0x62B5; // Sync Word (Sends 0xB5 then 0x62 on Little Endian)
    sim_output.command_voltage = state.command_voltage;
    sim_output.debug_flags = 0.0f;
    // Send non-biting (SimPacket_Output_t is 10 bytes now)
    HAL_UART_Transmit_IT(&huart2, (uint8_t*)&sim_output, sizeof(sim_output));
    
#else
    // Request a fresh sensor sample for next control cycle.
    CurrentSensor_SubmitSampleRequest();

	// A. Read latest cached hardware sample from sensor task.
	float raw_val = 0.0f;
    uint32_t sample_age_ms = 0U;
    if (!CurrentSensor_GetLatestSample(&raw_val, &sample_age_ms, HAL_GetTick())) {
        state.command_voltage = 0.0f;
        HBridge_SetVoltage(state.command_voltage, MAX_OUTPUT_VOLTAGE);
        return;
    }

	// B. Inject Sign (Your polarity logic)
	float signed_raw = (state.command_voltage < 0.0f) ? -1.0f * fabsf(raw_val) : fabsf(raw_val);

	// C. === LOW PASS FILTER (The Fix) ===
	// Logic: Keep 90% of the old smooth value, add only 10% of the new reading.
	// This ignores sudden spikes but tracks the true average.
	filtered_current = (filtered_current * 0.90f) + (signed_raw * 0.10f);

	// Update state with the CLEAN value
	state.measured_current = filtered_current;

	// D. Run PI on the SMOOTH value
	// Now the PI controller won't panic because the input is smooth.
	state.command_voltage = PI_Update(&pi_ctrl, state.target_current, state.measured_current);

    // Fail-safe: stale sample -> command safe zero output.
    if (sample_age_ms > 20U) {
        state.command_voltage = 0.0f;
    }
#endif
#endif

    // 3. Apply Command to H-Bridge
    HBridge_SetVoltage(state.command_voltage, MAX_OUTPUT_VOLTAGE);
    g_state_valid = 1U;
}

mtq_state_t InnerLoop_GetState(void)
{
    return state;
}

int InnerLoop_GetStateSnapshot(mtq_state_t *out, uint32_t timeout_ms)
{
    (void)timeout_ms;
    if (out == NULL || g_state_valid == 0U) {
        return 0;
    }
    *out = state;
    return 1;
}
