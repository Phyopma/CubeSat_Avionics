/* Application/Algorithms/Inc/inner_loop_control.h */
#ifndef INNER_LOOP_CONTROL_H
#define INNER_LOOP_CONTROL_H

#include <stdint.h>

// === CONFIGURATION: CONTROL MODE ===
// #define MTQ_MODE_OPEN_LOOP
typedef struct
{
    float target_current_x, target_current_y, target_current_z;
    float measured_current_x, measured_current_y, measured_current_z;
    float command_voltage_x, command_voltage_y, command_voltage_z;
} mtq_state_t;

// === API Functions ===
void InnerLoop_Init(void);
void InnerLoop_SetTargetCurrent(float x, float y, float z);
void InnerLoop_Update(void);          // Must be called at 1kHz
mtq_state_t InnerLoop_GetState(void); // for plotting
int InnerLoop_GetStateSnapshot(mtq_state_t *out, uint32_t timeout_ms);
void InnerLoop_Update_SimDataAvailable(void); // HITL Specific
void InnerLoop_SetVoltageLimit(float v_limit);
float InnerLoop_GetVoltageLimit(void);

#endif
