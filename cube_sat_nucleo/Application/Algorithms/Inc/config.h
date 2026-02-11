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

// Inertial Pointing (PD Controller)
// HITL tuned pointing defaults (flight-ready balance objective)
#define K_P                   0.00072658465f  // Proportional gain (Nm per rad error)
#define K_I                   0.000084290285f // Integral gain (Nm per rad-s error)
#define K_D                   0.14633999f     // Derivative gain (Nm per rad/s)
#define MAX_INTEGRAL_ERROR    1.0f     // Anti-windup limit (rad-s)

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

#endif // CONFIG_H
