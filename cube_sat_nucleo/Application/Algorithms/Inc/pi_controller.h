/* Application/Algorithms/Inc/pi_controller.h */
#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

typedef struct
{
    float Kp;         // Proportional Gain
    float Ki;         // Integral Gain
    float integrator; // Integral accumulator
    float limMin;     // Output Minimum
    float limMax;     // Output Maximum
    float T;          // Sample time in seconds
} PI_Config_t;

void PI_Init(PI_Config_t *pi, float Kp, float Ki, float T, float limit);
void PI_Reset(PI_Config_t *pi);
float PI_Update(PI_Config_t *pi, float setpoint, float measurement);

#endif