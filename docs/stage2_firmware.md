# Implementation Stage 2: Control Algorithms (STM32 Firmware)

**Goal:** Implement the ADCS state machine and control laws on the STM32. The outer loop determines the attitude mode and computes dipole requests; the inner loop drives magnetorquer currents accordingly.

**Status:** ✅ Complete (tuning ongoing)

## State Machine

```
┌──────────────┐  ω < 0.03 rad/s  ┌──────────────┐
│   DETUMBLE   │ ───────────────→ │   POINTING   │
│   (B-dot)    │ ←─────────────── │    (PID)     │
└──────────────┘  ω > 0.15 rad/s  └──────────────┘
       │
       ↓ (manual)
┌──────────────┐
│ SPIN STABLE  │
│ (Y-axis spin)│
└──────────────┘
```

Transitions are based on angular velocity magnitude (`Vec3_Norm(gyro)`) with hysteresis between 0.03 and 0.15 rad/s.

## Data Structures

```c
typedef enum {
    CTRL_MODE_DETUMBLE,     // B-Dot damping
    CTRL_MODE_SPIN_STABLE,  // Y-axis spin-up
    CTRL_MODE_POINTING      // Quaternion PID
} adcs_mode_t;

typedef struct {
    adcs_mode_t mode;
    float k_bdot, c_damp, i_virtual;
    float kp, ki, kd;
    vec3_t integral_error;
    quat_t q_target;       // Default: identity (nadir pointing)
} adcs_control_t;
```

## Mode A: Detumbling (Gyro-Based B-Dot)

**Algorithm:**

$$M = k_{bdot} \cdot (\omega \times B_{body})$$

- Uses gyro-estimated B-dot: $\dot{B}_{est} = \omega \times B$ (avoids noisy finite differencing)
- Gain `K_BDOT = 200,000` ensures magnetorquer saturation for aggressive damping (positive sign required for correct damping)
- **Singularity avoidance**: When $|\omega| > 0.1$ rad/s but $|\dot{B}_{est}| < 0.1 \cdot |\omega| \cdot |B|$, a kick (0.1 Am²) is applied on the axis **least aligned** with B to generate maximum torque

**Sign convention note:** Verified HITL: positive `k_bdot` produces correct damping behavior. Previously suggested negative sign was incorrect for this coordinate system.

## Mode B: Spin Stabilization

**Algorithm:** Simple Y-axis torque controller  

$$\tau_{req} = k_y \cdot (\omega_{target} - \omega_y) \cdot \hat{y}$$

- Target: ω_y = 0.5 rad/s (≈30°/s), k_y = 0.1
- Dipole projection: $M = \frac{B \times \tau_{req}}{|B|^2}$

## Mode C: Inertial Pointing (Quaternion PID)

**Algorithm:**

1. Compute target direction in body frame from quaternion:
   ```
   target_body = R(q)^T · [0, 0, 1]  (3rd column of DCM)
   ```

2. Error vector: `e = body_z × target_body`

3. **180° singularity check:** If `dot(body_z, target_body) < -0.999`, force `e = [0.1, 0, 0]` to break symmetry.

4. PID torque request:
   $$\tau_{req} = K_p \cdot e + K_i \cdot \int e \, dt - K_d \cdot \omega$$
   - Integral anti-windup: clamped to `MAX_INTEGRAL_ERROR = 1.0` rad·s

5. Magnetic projection:
   $$M = \frac{B \times \tau_{req}}{|B|^2}$$

## Pipeline: Outer Loop → Inner Loop → Actuators

```
OuterLoop_Update(sensors, &output, dt)
    → output.dipole_request = {mx, my, mz}  [Am²]

target_current = dipole_request * MTQ_DIPOLE_TO_AMP  [A]
    → MTQ_DIPOLE_TO_AMP = 1/2.88  (datasheet: 2.88 Am²/A)

InnerLoop_SetTargetCurrent(ix, iy, iz)
    → PI_Update(&pi_x, target, measured) → command_voltage

HBridge_SetVoltage(axis, voltage, max_voltage)
    → PWM output
```

## Key Configuration (`config.h`)

| Parameter | Value | Description |
|---|---|---|
| `K_BDOT` | 200,000 | B-dot gain (positive = damping) |
| `K_P` | 0.100 | Pointing proportional |
| `K_I` | 0.0001 | Pointing integral |
| `K_D` | 0.100 | Pointing derivative |
| `MAX_INTEGRAL_ERROR` | 1.0 | Anti-windup clamp |
| `DETUMBLE_OMEGA_THRESH` | 0.03 rad/s | Mode transition |
| `POINTING_OMEGA_THRESH` | 0.15 rad/s | Safety fallback |
| `MTQ_DIPOLE_TO_AMP` | 1/2.88 | Dipole → current (datasheet) |
| `PIL_MAX_VOLTAGE` | 3.3V | Supply voltage clamp (datasheet) |

## Deliverables

| File | Description |
|---|---|
| `outer_loop_control.c` | State machine + B-dot, spin, pointing controllers |
| `inner_loop_control.c` | PI current loop + HITL simulation support |
| `math_lib.c` | Vector cross/dot/norm, quaternion rotation |
| `config.h` | All tunable gains and thresholds |
