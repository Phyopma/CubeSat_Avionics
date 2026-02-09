#ifndef CONFIG_H
#define CONFIG_H

/* ========================================================================== */
/*   INNER LOOP CONFIGURATION (PI Controller)                                 */
/* ========================================================================== */

#define PIL_KP                5.0f
#define PIL_KI                1500.0f
#define PIL_T                 0.001f    // 1kHz loop
#define PIL_MAX_VOLTAGE       5.0f      // Absolute Max Limit (Power Team Requirement)
#define MTQ_COIL_RESISTANCE   28.0f     // Ohms (Datasheet: 28.0)

/* ========================================================================== */
/*   OUTER LOOP CONFIGURATION (ADCS Algorithms)                               */
/* ========================================================================== */

// Detumbling (B-Dot)
#define K_BDOT                5.0e6f    // Gain for M = -k * B_dot (Tuned for 0.5 Am^2 MTQs)

// Spin Stabilization (Kane Damper)
#define C_DAMP                0.05f     // Damping coefficient
#define I_VIRTUAL             0.01f     // Virtual inertia for damper

// Inertial Pointing (PD Controller)
// Tuned via simulation: kp=0.010, kd=0.300 gives best pointing stability in HITL
// Lower gains provide stability with magnetorquer-only actuation
#define K_P                   0.010f   // Proportional gain (Nm per rad error)
#define K_D                   0.300f   // Derivative gain (Nm per rad/s)

// State Machine Thresholds
#define DETUMBLE_OMEGA_THRESH 0.08f     // rad/s (Relaxed to enter Pointing sooner)
#define POINTING_OMEGA_THRESH 0.15f     // rad/s (Fallback to Detumble if omega increases)

// Actuator Conversion
// Note: B-dot law already has negative sign (M = -k * B_dot)
// This constant converts dipole (A*m^2) to current (A)
#define MTQ_DIPOLE_TO_AMP     (1.0f / 2.88f) // A per A*m^2 (Datasheet: 2.88 Am^2/A)

#endif // CONFIG_H
