#include "outer_loop_control.h"
#include "config.h"
#include <string.h>

static adcs_control_t ctrl;
static float dt = 0.01f; // 100Hz outer loop
static uint8_t force_mode_enabled = 0;
static adcs_mode_t forced_mode = CTRL_MODE_DETUMBLE;
static float detumble_ready_time = 0.0f;
static float pointing_unstable_time = 0.0f;
static float pointing_no_progress_time = 0.0f;
static float pointing_best_error_deg = 180.0f;

static void ResetModeTransitionState(void) {
    detumble_ready_time = 0.0f;
    pointing_unstable_time = 0.0f;
    pointing_no_progress_time = 0.0f;
    pointing_best_error_deg = 180.0f;
}

static float ComputePointingErrorDeg(quat_t q_curr) {
    float dot_z = 1.0f - 2.0f * (q_curr.x * q_curr.x + q_curr.y * q_curr.y);
    dot_z = fmaxf(-1.0f, fminf(1.0f, dot_z));
    return acosf(dot_z) * 57.2957795f;
}

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
    ctrl.w_damper = (vec3_t){0, 0, 0};
    ctrl.q_target = (quat_t){1.0f, 0.0f, 0.0f, 0.0f};
    force_mode_enabled = 0;
    forced_mode = CTRL_MODE_DETUMBLE;
    ResetModeTransitionState();
}

void OuterLoop_SetMode(adcs_mode_t mode) {
    if (ctrl.mode == mode) {
        return;
    }

    if (mode == CTRL_MODE_SPIN_STABLE) {
        ctrl.w_damper = (vec3_t){0, 0, 0};
    }

    // Integral state is only meaningful in Pointing mode.
    ctrl.integral_error = (vec3_t){0, 0, 0};

    ctrl.mode = mode;
    ResetModeTransitionState();
}

adcs_mode_t OuterLoop_GetMode(void) {
    return ctrl.mode;
}

void OuterLoop_SetForcedMode(uint8_t force_mode_code) {
    if (force_mode_code == 0) {
        force_mode_enabled = 0;
        return;
    }

    force_mode_enabled = 1;
    switch (force_mode_code) {
        case 1:
            forced_mode = CTRL_MODE_DETUMBLE;
            break;
        case 2:
            forced_mode = CTRL_MODE_SPIN_STABLE;
            ctrl.w_damper = (vec3_t){0, 0, 0};
            break;
        case 3:
            forced_mode = CTRL_MODE_POINTING;
            break;
        default:
            force_mode_enabled = 0;
            return;
    }

    OuterLoop_SetMode(forced_mode);
}

void OuterLoop_ResetControllerState(void) {
    ctrl.integral_error = (vec3_t){0, 0, 0};
    ctrl.w_damper = (vec3_t){0, 0, 0};
    ResetModeTransitionState();

    // Reset to a deterministic baseline mode unless an explicit override is active.
    if (force_mode_enabled) {
        OuterLoop_SetMode(forced_mode);
    } else {
        OuterLoop_SetMode(CTRL_MODE_DETUMBLE);
    }
}

void OuterLoop_SetSpinParams(float c_damp, float i_virtual) {
    ctrl.c_damp = fmaxf(c_damp, 1e-6f);
    ctrl.i_virtual = fmaxf(i_virtual, 1e-6f);
    ctrl.w_damper = (vec3_t){0, 0, 0};
}

void OuterLoop_SetGains(float k_bdot, float kp, float ki, float kd) {
    ctrl.k_bdot = k_bdot;
    ctrl.kp = kp;
    ctrl.ki = ki;
    ctrl.kd = kd;
    // Reset integral on gain change
    ctrl.integral_error = (vec3_t){0, 0, 0};
}

static vec3_t Control_BDot(vec3_t B, vec3_t w) {
    // Gyro-Based B-Dot: B_dot_body ~ -cross(w, B)
    // Damping Law: M = -k * B_dot = -k * (-cross(w, B)) = k * cross(w, B)
    vec3_t B_dot_estimated = Vec3_Cross(w, B);
    vec3_t m_cmd = Vec3_ScalarMult(B_dot_estimated, ctrl.k_bdot);
    
    // Singularity Avoidance (Kick)
    // If satellite is spinning fast but B_dot is small, we are aligned with B-field.
    // Apply a geometric kick to break alignment.
    float w_norm = Vec3_Norm(w);
    float b_dot_norm = Vec3_Norm(B_dot_estimated);
    
    // Thresholds: Spin > 0.1 rad/s (approx 6 deg/s), but normalized B_dot is small
    // "Small" B_dot relative to expected max (w * B_mag)
    float b_mag = Vec3_Norm(B);
    float expected_b_dot = w_norm * b_mag;
    
    if (w_norm > 0.1f && b_dot_norm < 0.1f * expected_b_dot) {
        // We are spinning but not generating B_dot -> Aligned!
        // Apply kick on an axis perpendicular to B to generate torque
        // Choose axis least aligned with B for maximum torque
        vec3_t kick_axis;
        float abs_bx = fabsf(B.x), abs_by = fabsf(B.y), abs_bz = fabsf(B.z);
        if (abs_bx <= abs_by && abs_bx <= abs_bz) {
            kick_axis = (vec3_t){1.0f, 0.0f, 0.0f}; // X least aligned
        } else if (abs_by <= abs_bz) {
            kick_axis = (vec3_t){0.0f, 1.0f, 0.0f}; // Y least aligned
        } else {
            kick_axis = (vec3_t){0.0f, 0.0f, 1.0f}; // Z least aligned
        }
        m_cmd = Vec3_Add(m_cmd, Vec3_ScalarMult(kick_axis, 0.1f));
    }

    return m_cmd;
}

static vec3_t Control_SpinStabilization(vec3_t B, vec3_t w) {
    // Virtual Kane Damper:
    // tau_d = c * (w_body - w_damper)
    // tau_req = -Proj_perp_B(tau_d), then map to dipole via m = (B x tau_req) / |B|^2
    float B2 = Vec3_Dot(B, B);
    if (B2 < 1e-12f) return (vec3_t){0, 0, 0};

    float inv_b_mag = 1.0f / sqrtf(B2);
    vec3_t b_hat = Vec3_ScalarMult(B, inv_b_mag);

    vec3_t rel_w = Vec3_Sub(w, ctrl.w_damper);
    vec3_t tau_d = Vec3_ScalarMult(rel_w, ctrl.c_damp);

    // Project tau_d onto plane normal to B, then negate for dissipative action.
    float tau_parallel = Vec3_Dot(tau_d, b_hat);
    vec3_t tau_perp = Vec3_Sub(tau_d, Vec3_ScalarMult(b_hat, tau_parallel));
    vec3_t tau_req = Vec3_ScalarMult(tau_perp, -1.0f);

    // Damper state: I_d * w_d_dot = c * (w - w_damper)
    float gain = ctrl.c_damp / fmaxf(ctrl.i_virtual, 1e-6f);
    vec3_t w_d_dot = Vec3_ScalarMult(rel_w, gain);
    ctrl.w_damper = Vec3_Add(ctrl.w_damper, Vec3_ScalarMult(w_d_dot, dt));

    // Guard against numerical blow-up from invalid parameter injections.
    float wd_mag = Vec3_Norm(ctrl.w_damper);
    if (wd_mag > 10.0f) {
        ctrl.w_damper = Vec3_ScalarMult(ctrl.w_damper, 10.0f / wd_mag);
    }

    return Vec3_ScalarMult(Vec3_Cross(B, tau_req), 1.0f / B2);
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
    
    // Check for 180 degree singularity
    float dot_z = Vec3_Dot(current_z, target_body);
    vec3_t err_vec;
    if (dot_z < -0.999f) {
        // Antiparallel! Cross product is zero. 
        // Apply a small geometric kick on X-axis to break symmetry.
        err_vec = (vec3_t){0.1f, 0.0f, 0.0f}; 
    } else {
        err_vec = Vec3_Cross(current_z, target_body);
    }
    
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
    if (force_mode_enabled) {
        OuterLoop_SetMode(forced_mode);
        return;
    }

    float omega_norm = Vec3_Norm(input->gyro);
    float dt_step = fmaxf(dt, 1e-4f);

    switch (ctrl.mode) {
        case CTRL_MODE_DETUMBLE:
            if (omega_norm < DETUMBLE_OMEGA_THRESH) {
                detumble_ready_time += dt_step;
                if (detumble_ready_time >= DETUMBLE_ENTRY_HOLD_SEC) {
                    // Tumbling has stayed low-rate long enough, switch to Pointing.
                    OuterLoop_SetMode(CTRL_MODE_POINTING);
                }
            } else {
                detumble_ready_time = 0.0f;
            }
            break;

        case CTRL_MODE_SPIN_STABLE:
            // Placeholder: Logic to transition from Spin to Pointing
            break;

        case CTRL_MODE_POINTING: {
            float pointing_error_deg = ComputePointingErrorDeg(input->orientation);
            if (pointing_error_deg + POINTING_PROGRESS_EPS_DEG < pointing_best_error_deg) {
                pointing_best_error_deg = pointing_error_deg;
                pointing_no_progress_time = 0.0f;
            } else {
                pointing_no_progress_time += dt_step;
            }

            float omega_fallback_thresh = POINTING_OMEGA_THRESH;
            if (pointing_error_deg > POINTING_LARGE_ERR_DEG) {
                omega_fallback_thresh = fminf(omega_fallback_thresh, POINTING_HIGH_ERR_OMEGA_THRESH);
            }

            if (omega_norm > omega_fallback_thresh) {
                pointing_unstable_time += dt_step;
            } else {
                pointing_unstable_time = 0.0f;
            }

            if (pointing_unstable_time >= POINTING_EXIT_HOLD_SEC ||
                (pointing_no_progress_time >= POINTING_NO_PROGRESS_SEC &&
                 pointing_error_deg > POINTING_STALL_ERR_DEG &&
                 omega_norm > DETUMBLE_OMEGA_THRESH)) {
                // Pointing became unstable or stalled; fall back to Detumble.
                OuterLoop_SetMode(CTRL_MODE_DETUMBLE);
            }
            break;
        }

        default:
            break;
    }
}

void OuterLoop_Update(adcs_sensor_input_t *input, adcs_output_t *output, float dt_in) {
    // Update local dt for integrate calculations
    dt = dt_in;
    
    // 1. Run State Machine
    Update_State_Machine(input);

    vec3_t m_cmd = {0, 0, 0};
    
    switch (ctrl.mode) {
        case CTRL_MODE_DETUMBLE:
            m_cmd = Control_BDot(input->mag_field, input->gyro);
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
