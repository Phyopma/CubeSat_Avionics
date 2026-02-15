/* Application/Algorithms/Inc/inner_loop_control.h */
#ifndef INNER_LOOP_CONTROL_H
#define INNER_LOOP_CONTROL_H

#include <stdint.h>

// === CONFIGURATION: CONTROL MODE ===
//#define MTQ_MODE_OPEN_LOOP
typedef struct
{
    float target_current;
    float measured_current;
    float command_voltage;
} mtq_state_t;

// === CONFIGURATION: HARDWARE CONSTANTS ===
#define MTQ_COIL_RESISTANCE 10.0f // Ohms (Measure with multimeter)
//#define HBRIDGE_SUPPLY_VOLTS 5.0f // Volts (Measure at VCC pin)
// Hardware Constants for Tuning
// When testing with 5V, max output is 5.0V
#define MAX_OUTPUT_VOLTAGE 5.0f

// === API Functions ===
void InnerLoop_Init(void);
void InnerLoop_SetTargetCurrent(float current_amps);
int InnerLoop_SetTargetCurrentAsync(float current_amps, uint32_t timeout_ms);
void InnerLoop_Update(void);          // Must be called at 1kHz
mtq_state_t InnerLoop_GetState(void); // for plotting
int InnerLoop_GetStateSnapshot(mtq_state_t *out, uint32_t timeout_ms);

#endif
