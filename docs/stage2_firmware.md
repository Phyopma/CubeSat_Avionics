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
    CTRL_MODE_IDLE,
    CTRL_MODE_DETUMBLE,     // B-Dot damping
    CTRL_MODE_SPIN_STABLE,  // Kane virtual damper
    CTRL_MODE_POINTING      // Quaternion PID
} adcs_mode_t;

typedef struct {
    adcs_mode_t mode;
    float k_bdot, c_damp, i_virtual;
    float kp, ki, kd;
    vec3_t integral_error;
    vec3_t w_damper;
    quat_t q_target;       // Default: identity (nadir pointing)
} adcs_control_t;
```

## Mode A: Detumbling (Gyro-Based B-Dot)

**Algorithm:**

$$M = k_{bdot} \cdot (\omega \times B_{body})$$

- Uses gyro-estimated B-dot: $\dot{B}_{est} = \omega \times B$ (avoids noisy finite differencing)
- Gain `K_BDOT = 400,000` provides strong damping authority in current HITL tuning (positive sign required for correct damping)
- **Singularity avoidance**: When $|\omega| > 0.1$ rad/s but $|\dot{B}_{est}| < 0.1 \cdot |\omega| \cdot |B|$, a kick (0.1 Am²) is applied on the axis **least aligned** with B to generate maximum torque

**Sign convention note:** Verified HITL: positive `k_bdot` produces correct damping behavior. Previously suggested negative sign was incorrect for this coordinate system.

## Mode B: Spin Stabilization (Virtual Kane Damper)

**Algorithm:** virtual viscous damper + magnetic torque projection

$$\tau_d = c_{damp}(\omega - \omega_d)$$
$$\dot{\omega}_d = \frac{c_{damp}}{I_d}(\omega - \omega_d)$$
$$\tau_{req} = -\operatorname{Proj}_{\perp B}(\tau_d)$$
$$M = \frac{B \times \tau_{req}}{|B|^2}$$

- `c_damp`: virtual damping coefficient
- `i_virtual (I_d)`: virtual damper inertia
- `w_damper`: internal virtual damper angular-rate state

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
    → In HITL, runtime dipole mapping can be overridden from host packet

InnerLoop_SetTargetCurrent(ix, iy, iz)
    → PI_Update(&pi_x, target, measured) → command_voltage

HBridge_SetVoltage(axis, voltage, max_voltage)
    → PWM output
```

## HITL Runtime Calibration Override (Auto)

In `SIMULATION_MODE`, firmware consumes two runtime calibration fields from each host packet:

- `max_voltage_mV` (`uint16`)
- `dipole_strength_milli` (`uint16`)

Behavior:

1. Runtime voltage clamp is applied through inner loop API:
   - `InnerLoop_SetVoltageLimit(max_voltage_mV * 1e-3)`
2. Runtime dipole conversion is applied in main loop:
   - `mtq_dipole_to_amp_runtime = 1.0 / (dipole_strength_milli * 1e-3)`
3. No extra enable flag is used; override is always active in HITL packet flow.

Host defaults when CLI overrides are omitted:

- `max_voltage = 3.3 V`
- `dipole_strength = 2.88 Am²/A`

Packet schema note:

- Input packet format is now `<H3f3f3f4f5fBHH>`
- Input packet size is now `79` bytes
- Host and firmware must be version-matched.

## Key Configuration (`config.h`)

| Parameter | Value | Description |
|---|---|---|
| `K_BDOT` | 400,000 | B-dot gain (positive = damping) |
| `C_DAMP` | 0.0016405224 | Kane virtual damping coefficient |
| `I_VIRTUAL` | 0.0083109405 | Kane virtual damper inertia |
| `K_P` | 0.0012 | Pointing proportional |
| `K_I` | 0.00003 | Pointing integral |
| `K_D` | 0.17 | Pointing derivative |
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
