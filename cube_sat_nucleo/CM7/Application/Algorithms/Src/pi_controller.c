/* Application/Algorithms/Src/pi_controller.c */
#include "pi_controller.h"

void PI_Init(PI_Config_t *pi, float Kp, float Ki, float T, float limit)
{
    pi->Kp = Kp;
    pi->Ki = Ki;
    pi->T = T;
    pi->limMax = limit;
    pi->limMin = -limit;
    pi->integrator = 0.0f;
}

void PI_Reset(PI_Config_t *pi)
{
    pi->integrator = 0.0f;
}

float PI_Update(PI_Config_t *pi, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    // Proportional term
    float P = pi->Kp * error;

    // Integral term (Trapezoidal rule)
    pi->integrator += 0.5f * pi->Ki * pi->T * (error + error);

    // Anti-Windup (Clamp Integrator)
    if (pi->integrator > pi->limMax)
        pi->integrator = pi->limMax;
    if (pi->integrator < pi->limMin)
        pi->integrator = pi->limMin;

    // Total Output
    float out = P + pi->integrator;

    // Output Saturation
    if (out > pi->limMax)
        out = pi->limMax;
    if (out < pi->limMin)
        out = pi->limMin;

    return out;
}