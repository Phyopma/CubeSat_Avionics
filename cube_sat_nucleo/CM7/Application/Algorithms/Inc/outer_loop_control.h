#ifndef OUTER_LOOP_CONTROL_H
#define OUTER_LOOP_CONTROL_H

#include "math_lib.h"
#include <stdint.h>

typedef enum {
    CTRL_MODE_IDLE,
    CTRL_MODE_DETUMBLE,
    CTRL_MODE_SPIN_STABLE,
    CTRL_MODE_POINTING
} adcs_mode_t;

typedef struct {
    adcs_mode_t mode;
    float k_bdot;
    float c_damp;
    float i_virtual;
    float kp;               // Proportional Gain
    float ki;               // Integral Gain
    float kd;               // Derivative Gain
    vec3_t integral_error;  // Integrated error state
    vec3_t w_damper;        // Virtual damper angular rate state
    quat_t q_target;
} adcs_control_t;

typedef struct {
    vec3_t mag_field;       // Tesla
    vec3_t gyro;            // rad/s
    quat_t orientation;
} adcs_sensor_input_t;

typedef struct {
    vec3_t dipole_request;  // A*m^2
} adcs_output_t;

void OuterLoop_Init(void);
void OuterLoop_Update(adcs_sensor_input_t *input, adcs_output_t *output, float dt);
void OuterLoop_SetMode(adcs_mode_t mode);
adcs_mode_t OuterLoop_GetMode(void);
uint8_t OuterLoop_GetTelemetryByte(void);
void OuterLoop_SetForcedMode(uint8_t force_mode_code);
void OuterLoop_ResetControllerState(void);
void OuterLoop_SetSpinParams(float c_damp, float i_virtual);

/**
 * @brief Update control gains dynamically (e.g. from telemetry/sim)
 * @param k_bdot Detumble Gain
 * @param kp Pointing Proportional Gain
 * @param ki Pointing Integral Gain
 * @param kd Pointing Derivative Gain
 */
void OuterLoop_SetGains(float k_bdot, float kp, float ki, float kd);

#endif // OUTER_LOOP_CONTROL_H
