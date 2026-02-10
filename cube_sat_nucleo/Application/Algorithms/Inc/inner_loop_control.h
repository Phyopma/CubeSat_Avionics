/* Application/Algorithms/Inc/inner_loop_control.h */
#ifndef INNER_LOOP_CONTROL_H
#define INNER_LOOP_CONTROL_H

#include <stdint.h>

// === CONFIGURATION: CONTROL MODE ===
//#define MTQ_MODE_OPEN_LOOP
typedef struct
{
    float target_current_x, target_current_y, target_current_z;
    float measured_current_x, measured_current_y, measured_current_z;
    float command_voltage_x, command_voltage_y, command_voltage_z;
} mtq_state_t;

// === CONFIGURATION: HARDWARE CONSTANTS ===
// #define MTQ_COIL_RESISTANCE 10.0f // Moved to config.h
//#define HBRIDGE_SUPPLY_VOLTS 5.0f // Volts (Measure at VCC pin)
// Hardware Constants for Tuning
// Datasheet typical supply: 3.3V
#define MAX_OUTPUT_VOLTAGE 3.3f

// === API Functions ===
void InnerLoop_Init(void);
void InnerLoop_SetTargetCurrent(float x, float y, float z);
void InnerLoop_Update(void);          // Must be called at 1kHz
mtq_state_t InnerLoop_GetState(void); // for plotting
void InnerLoop_Update_SimDataAvailable(void); // HITL Specific

#endif
