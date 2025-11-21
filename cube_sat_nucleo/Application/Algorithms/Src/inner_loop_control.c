/* Application/Algorithms/Src/inner_loop_control.c */
#include "inner_loop_control.h"
#include "hbridge.h"
#include "current_sensor.h"
#include "pi_controller.h"

// Private State Variables
float target_current = 0.0f;
PI_Config_t pi_ctrl;
static mtq_state_t state;

// for tuning the PI controller (Live Expression in debugger)
extern float current_measure;
extern float cmd_voltage;

void InnerLoop_PrintTelemetry(char *buffer)
{
    // Format: "Target,Measured,Voltage\r\n"
    // Multiplied by 1000 to print as integers (mA) avoids float %f issues
    sprintf(buffer, "%d,%d,%d\r\n",
            (int)(target_current * 1000),
            (int)(current_measure * 1000), // Will be 0 in Open Loop
            (int)(cmd_voltage * 1000));
}
// tuning the PI controller

void InnerLoop_Init(void)
{
    // 1. Initialize Drivers
    HBridge_Init();
    CurrentSensor_Init();

    // 2. Initialize PI Controller (Always init, even if unused right now)
    // Kp=5.0, Ki=100.0, T=0.001s (1kHz), Output Limit = Supply Voltage
    PI_Init(&pi_ctrl, 5.0f, 100.0f, 0.001f, HBRIDGE_SUPPLY_VOLTS);
}

void InnerLoop_SetTargetCurrent(float current_amps)
{
    target_current = current_amps;
}

// This function runs inside the TIM6 Interrupt (1kHz)
void InnerLoop_Update(void)
{
    float cmd_voltage = 0.0f;

#ifdef MTQ_MODE_OPEN_LOOP
    // === MODE A: OPEN LOOP (Active) ===
    // Physics Model: V = I * R
    // This ignores the sensor and assumes resistance is constant.
    cmd_voltage = target_current * MTQ_COIL_RESISTANCE;

#else
    // === MODE B: CLOSED LOOP (Future) ===
    // 1. Read Feedback
    float actual_current = CurrentSensor_Read_Amps();

    // 2. Run PI Algorithm
    // This calculates voltage needed to minimize error.
    cmd_voltage = PI_Update(&pi_ctrl, target_current, actual_current);
#endif

    // 3. Apply Command to H-Bridge
    HBridge_SetVoltage(cmd_voltage, HBRIDGE_SUPPLY_VOLTS);
}

mtq_state_t InnerLoop_GetState(void)
{
    return state;
}