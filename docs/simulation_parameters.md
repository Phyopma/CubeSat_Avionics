# Simulation Parameters & Tuning Guide

Reference for all tunable parameters in the HITL simulation. Parameters can be adjusted via CLI flags or by editing source files.

## 1. Satellite Mass Properties

**File:** `physics_engine/physics.py` → `SatellitePhysics.__init__`

| Parameter | Value | Unit | Description |
|---|---|---|---|
| `I_xx` | 0.00833 | kg·m² | Moment of inertia (X-axis, 2U long axis) |
| `I_yy` | 0.00833 | kg·m² | Moment of inertia (Y-axis) |
| `I_zz` | 0.00333 | kg·m² | Moment of inertia (Z-axis, 2U short axis) |

Based on a 2U CubeSat (10×10×20 cm, 2 kg). Adjust for different form factors:
- **1U:** `[0.002, 0.002, 0.002]`
- **3U:** `[0.023, 0.023, 0.003]`

## 2. Magnetorquer Electrical Properties

**File:** `physics_engine/physics.py` → `SatellitePhysics.__init__`

| Parameter | Value | Unit | Description |
|---|---|---|---|
| `R` | 28.0 | Ω | Coil resistance (all axes identical) |
| `L` (X, Y) | 0.025 | H | Coil inductance (25mH, longer coils) |
| `L` (Z) | 0.012 | H | Coil inductance (12mH, shorter coil) |
| Dipole factor | 2.88 | Am²/A | Effective magnetic dipole per amp (datasheet: 0.34 Am² @ 3.3V, R=28Ω) |

**RL Time Constants:** τ_xy = L/R = 0.89ms, τ_z = 0.43ms. The RK4 integrator automatically sub-steps to ≤0.5ms intervals for numerical stability.

## 3. Environmental Model

**File:** `physics_engine/physics.py` → `get_magnetic_field_inertial`

| Parameter | Value | Description |
|---|---|---|
| B₀ | 50 µT | Earth's field magnitude |
| Orbit period | 5400s (90 min) | LEO orbital period |
| Inclination | 51.6° | ISS-like orbit |
| Dipole tilt | 11.7° | Earth's magnetic axis tilt |
| Earth rotation | 86400s | Sidereal day period |

The model uses a **tilted dipole** approximation: the satellite orbits in an inclined plane while Earth's magnetic axis rotates. This produces a realistic time-varying B-field in all three body axes.

## 4. Control Gains

**File:** `cube_sat_nucleo/Application/Algorithms/Inc/config.h`
**Runtime override:** CLI flags `--kbdot`, `--kp`, `--ki`, `--kd`

| Gain | Default | Mode | Effect |
|---|---|---|---|
| `K_BDOT` | 200,000 | Detumble | Positive = damping; higher magnitude = faster saturation |
| `K_P` | 0.100 | Pointing | Higher = stiffer spring toward target |
| `K_I` | 0.0001 | Pointing | Higher = eliminates steady-state error (risk of windup) |
| `K_D` | 0.100 | Pointing | Higher = more damping (slower convergence) |

### Inner Loop (PI Current Controller)

**File:** `config.h`

| Gain | Value | Description |
|---|---|---|
| `PIL_KP` | 5.0 | Proportional gain |
| `PIL_KI` | 1500.0 | Integral gain |
| `PIL_T` | 0.001 | Sample time (1kHz) |
| `PIL_MAX_VOLTAGE` | 3.3V | Output clamp (datasheet typical supply) |

## 5. State Machine Thresholds

| Threshold | Value | Transition |
|---|---|---|
| `DETUMBLE_OMEGA_THRESH` | 0.03 rad/s | Detumble → Pointing |
| `POINTING_OMEGA_THRESH` | 0.15 rad/s | Pointing → Detumble (safety) |

## 6. Simulation Timing

**CLI flags:** `--dt`, `--realtime`

| Mode | Step Size | Speed | Best For |
|---|---|---|---|
| Fast (default) | 0.1s | ~10 min/orbit | Quick iteration |
| Realtime | 1.0s | 90 min/orbit | Realistic timing |
| Custom | `--dt <val>` | Varies | Precision testing |

The physics engine automatically sub-steps the RL circuit dynamics within each outer step to maintain numerical stability.

## Stress Testing Scenarios

```bash
# 1. Weak Actuator (reduce dipole effectiveness)
#    Edit physics.py: m_actual = i * 1.0 (instead of 2.88)

# 2. High Inertia (heavy payload)
#    Edit physics.py: self.I = np.diag([0.017, 0.017, 0.007])

# 3. Extreme Tumble
uv run simulation_host.py --scenario detumble --initial_omega 3.0 3.0 3.0

# 4. 180° Singularity Recovery
uv run simulation_host.py --scenario pointing --initial_omega 0.01 0.01 0.01

# 5. Open Loop Verification (bypass PI)
uv run simulation_host.py --scenario detumble --open-loop --debug
```
