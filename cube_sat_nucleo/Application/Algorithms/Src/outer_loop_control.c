#include "outer_loop_control.h"
#include "config.h"
#include <string.h>

static adcs_control_t ctrl;
static vec3_t last_mag_field = {0, 0, 0};
static float dt = 0.01f; // 100Hz outer loop

void OuterLoop_Init(void) {
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.mode = CTRL_MODE_DETUMBLE;
    ctrl.k_bdot = K_BDOT;
    ctrl.c_damp = C_DAMP;
    ctrl.i_virtual = I_VIRTUAL;
    ctrl.kp = K_P;
    ctrl.ki = K_I;
    ctrl.kd = K_D;
    ctrl.integral_error = (vec3_t){0, 0, 0};
    ctrl.q_target = (quat_t){1.0f, 0.0f, 0.0f, 0.0f};
}

void OuterLoop_SetMode(adcs_mode_t mode) {
    ctrl.mode = mode;
}

adcs_mode_t OuterLoop_GetMode(void) {
    return ctrl.mode;
}

void OuterLoop_SetGains(float k_bdot, float kp, float ki, float kd) {
    ctrl.k_bdot = k_bdot;
    ctrl.kp = kp;
    ctrl.ki = ki;
    ctrl.kd = kd;
    // Reset integral on gain change
    ctrl.integral_error = (vec3_t){0, 0, 0};
}

static vec3_t Control_BDot(vec3_t B) {
    static int first_run = 1;
    if (first_run) {
        last_mag_field = B;
        first_run = 0;
        return (vec3_t){0, 0, 0};
    }

    // B_dot approx = (B_now - B_last) / dt
    vec3_t B_dot = Vec3_ScalarMult(Vec3_Sub(B, last_mag_field), 1.0f / dt);
    last_mag_field = B;
    
    // M = -k * B_dot
    return Vec3_ScalarMult(B_dot, -ctrl.k_bdot);
}

static vec3_t Control_SpinStabilization(vec3_t B, vec3_t w) {
    // Virtual Kane Damper logic
    // Implementation of M = (1/B^2) * (tau_req x B)
    // tau_req = -c_damp * w_damper * z_hat
    
    // Update virtual damper state (simplified)
    // dW_d = (1/I_v) * (tau_mag . z_hat - c_damp * W_d)
    // For now, simple proportional to w_z
    float tau_req_z = -ctrl.c_damp * w.z;
    vec3_t tau_req = {0, 0, tau_req_z};
    
    float B2 = Vec3_Dot(B, B);
    if (B2 < 1e-12f) return (vec3_t){0, 0, 0};
    
    return Vec3_ScalarMult(Vec3_Cross(tau_req, B), 1.0f / B2);
}

static vec3_t Control_Pointing(quat_t q_curr, vec3_t w, vec3_t B) {
    // Convert current quaternion to target vector in body frame.
    // For q_target = identity, target is Inertial Z-axis.
    // target_body = R^T * [0, 0, 1] (3rd column of DCM)
    float qw = q_curr.w, qx = q_curr.x, qy = q_curr.y, qz = q_curr.z;
    vec3_t target_body = {
        2.0f * (qx*qz - qw*qy),
        2.0f * (qy*qz + qw*qx),
        1.0f - 2.0f * (qx*qx + qy*qy)
    };
    
    // Nadir pointing: align body Z-axis with target
    vec3_t current_z = {0.0f, 0.0f, 1.0f};
    vec3_t err_vec = Vec3_Cross(current_z, target_body);
    
    // PID Controller: tau = kp * err + ki * int_err - kd * w
    
    // Update Integral Error with Anti-Windup
    ctrl.integral_error = Vec3_Add(ctrl.integral_error, Vec3_ScalarMult(err_vec, dt));
    
    // Clamp Integral Error
    float int_mag = Vec3_Norm(ctrl.integral_error);
    if (int_mag > MAX_INTEGRAL_ERROR) {
        ctrl.integral_error = Vec3_ScalarMult(ctrl.integral_error, MAX_INTEGRAL_ERROR / int_mag);
    }
    
    vec3_t tau_p = Vec3_ScalarMult(err_vec, ctrl.kp);
    vec3_t tau_i = Vec3_ScalarMult(ctrl.integral_error, ctrl.ki);
    vec3_t tau_d = Vec3_ScalarMult(w, ctrl.kd);
    
    vec3_t tau_req = Vec3_Sub(Vec3_Add(tau_p, tau_i), tau_d);
    
    float B2 = Vec3_Dot(B, B);
    if (B2 < 1e-12f) return (vec3_t){0, 0, 0};
    
    return Vec3_ScalarMult(Vec3_Cross(B, tau_req), 1.0f / B2);
}

static void Update_State_Machine(adcs_sensor_input_t *input) {
    float omega_norm = Vec3_Norm(input->gyro);

    switch (ctrl.mode) {
        case CTRL_MODE_DETUMBLE:
            if (omega_norm < DETUMBLE_OMEGA_THRESH) {
                // Tumbling has reduced enough, switch to Pointing
                OuterLoop_SetMode(CTRL_MODE_POINTING);
            }
            break;

        case CTRL_MODE_SPIN_STABLE:
            // Placeholder: Logic to transition from Spin to Pointing
            break;

        case CTRL_MODE_POINTING:
            if (omega_norm > POINTING_OMEGA_THRESH) {
                // Tumbling is too high, safety fallback to Detumble
                OuterLoop_SetMode(CTRL_MODE_DETUMBLE);
            }
            break;

        default:
            break;
    }
}

void OuterLoop_Update(adcs_sensor_input_t *input, adcs_output_t *output) {
    // 1. Run State Machine
    Update_State_Machine(input);

    vec3_t m_cmd = {0, 0, 0};
    
    switch (ctrl.mode) {
        case CTRL_MODE_DETUMBLE:
            m_cmd = Control_BDot(input->mag_field);
            break;
        case CTRL_MODE_SPIN_STABLE:
            m_cmd = Control_SpinStabilization(input->mag_field, input->gyro);
            break;
        case CTRL_MODE_POINTING:
            m_cmd = Control_Pointing(input->orientation, input->gyro, input->mag_field);
            break;
        default:
            break;
    }
    
    output->dipole_request = m_cmd;
}
