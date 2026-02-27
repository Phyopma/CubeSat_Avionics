#ifndef CONFIG_H
#define CONFIG_H

/* ========================================================================== */
/*   INNER LOOP CONFIGURATION (PI Controller)                                 */
/* ========================================================================== */

#define PIL_KP                5.0f
#define PIL_KI                1500.0f
#define PIL_T                 0.001f    // 1kHz loop
#define PIL_MAX_VOLTAGE       3.3f      // Datasheet typical supply voltage
#define MTQ_COIL_RESISTANCE   28.0f     // Ohms (Datasheet: 28.0)
#define MTQ_COIL_IND_XY       0.025f    // Henries (25mH typical)
#define MTQ_COIL_IND_Z        0.012f    // Henries (12mH typical)

/* ========================================================================== */
/*   OUTER LOOP CONFIGURATION (ADCS Algorithms)                               */
/* ========================================================================== */

// Detumbling (B-Dot)
// Positive sign required for damping: M = k * cross(w, B)
// Verified HITL: positive → DAMPING, negative → SPIN-UP.
#define K_BDOT                400000.0f   // HITL tuned gain for Gyro-based B-dot

// Spin Stabilization (Kane Damper)
#define C_DAMP                0.0016405224f  // HITL tuned Kane damping coefficient
#define I_VIRTUAL             0.0083109405f  // HITL tuned virtual inertia for damper

// Inertial Pointing (PD/PID Controller)
// Locked baseline for reproducible HITL comparisons.
#define K_P                   0.0012f   // Proportional gain (Nm per rad error)
#define K_I                   0.00003f  // Integral gain (Nm per rad-s error)
#define K_D                   0.17f     // Derivative gain (Nm per rad/s)
#define MAX_INTEGRAL_ERROR    1.0f      // Anti-windup limit (rad-s)

// Pointing integrator management (anti-windup + conditional integration)
#define POINTING_INT_LEAK               0.08f   // 1/s integral bleed when not integrating
#define POINTING_INT_ENABLE_ERR_DEG     60.0f   // Integrate only below this pointing error
#define POINTING_INT_ENABLE_OMEGA       0.08f   // Integrate only below this body-rate norm

// Pointing gain scheduling for large-angle recovery
#define POINTING_SCHED_ERR_LOW_DEG      12.0f   // Below this angle use low-angle gains
#define POINTING_SCHED_ERR_HIGH_DEG     80.0f   // Above this angle use high-angle gains
#define POINTING_KP_SCALE_LOW           1.35f   // Kp multiplier in low-angle regime
#define POINTING_KD_SCALE_HIGH          1.90f   // Kd multiplier in high-angle regime

// State Machine Thresholds
#define DETUMBLE_OMEGA_THRESH 0.03f     // rad/s (Relaxed to enter Pointing sooner)
#define POINTING_OMEGA_THRESH 0.15f     // rad/s (Fallback to Detumble if omega increases)
#define DETUMBLE_ENTRY_HOLD_SEC        2.0f    // Require low-rate condition this long before Pointing
#define POINTING_EXIT_HOLD_SEC         1.5f    // Require unstable condition this long before fallback
#define POINTING_LARGE_ERR_DEG         75.0f   // Large-angle regime with tighter rate margin
#define POINTING_HIGH_ERR_OMEGA_THRESH 0.05f   // Fallback threshold during large-angle pointing
#define POINTING_PROGRESS_EPS_DEG      0.25f   // Minimum error decrease counted as progress
#define POINTING_NO_PROGRESS_SEC       45.0f   // Timeout before fallback when Pointing stalls
#define POINTING_STALL_ERR_DEG         45.0f   // Stall fallback active only above this error

// Actuator Conversion
// B-dot convention in this codebase:
// B_dot_est = cross(w, B), M = k_bdot * B_dot_est with k_bdot > 0 for damping.
// This constant converts dipole (A*m^2) to current (A)
#define MTQ_DIPOLE_TO_AMP     (1.0f / 2.88f) // Datasheet: 2.88 Am^2/A (0.34 Am² @ 3.3V, R=28Ω)

// Simulation telemetry quantization (HITL packet v2)
// Values are encoded as int16 Q15 fractions of these full-scale magnitudes.
#define TELEMETRY_PACKET_VERSION 1u
#define M_CMD_FULL_SCALE_AM2     8.0f
#define TAU_FULL_SCALE_NM        0.05f

#endif // CONFIG_H
