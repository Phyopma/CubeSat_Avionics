# Simulation Parameters & Tuning Guide

Reference for all tunable parameters in the HITL simulation. Parameters can be adjusted via CLI flags or by editing source files.

## 1. Satellite Mass Properties

**Source file:** `physics_engine/config/structural_constants_w26.json`  
**Loader:** `physics_engine/constants_loader.py`  
**Consumer:** `physics_engine/physics.py` → `SatellitePhysics.__init__`

| Parameter | Value | Unit | Description |
|---|---|---|---|
| `Px` | 0.01186725757 | kg·m² | Principal moment X (converted from g·mm²) |
| `Py` | 0.01257530376 | kg·m² | Principal moment Y (converted from g·mm²) |
| `Pz` | 0.01286669027 | kg·m² | Principal moment Z (converted from g·mm²) |

Principal-axis directions are normalized/orthogonalized, then used to build the full body-frame tensor:

`I_body = R * diag(Px, Py, Pz) * R^T`

The loader validates symmetry and positive-definiteness before simulation starts.

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
| Orbit period | 5739s (95.65 min) | W26 source value |
| Inclination | 97.5977° | W26 source value |
| RAAN | 41.2° | W26 source note (terminator reference) |
| Semimajor axis | 6,928,140 m | W26 source value |
| Dipole tilt | 11.7° | Earth's magnetic axis tilt |
| Earth rotation | 86400s | Sidereal day period |

The model uses a **tilted dipole** approximation: the satellite orbits in an inclined plane while Earth's magnetic axis rotates. This produces a realistic time-varying B-field in all three body axes.

## 4. Control Gains

**File:** `cube_sat_nucleo/Application/Algorithms/Inc/config.h`
**Runtime override:** CLI flags `--kbdot`, `--kp`, `--ki`, `--kd`

| Gain | Default | Mode | Effect |
|---|---|---|---|
| `K_BDOT` | 400,000 | Detumble | Positive = damping; higher magnitude = faster saturation |
| `K_P` | 0.0012 | Pointing | Higher = stiffer spring toward target |
| `K_I` | 0.00003 | Pointing | Higher = eliminates steady-state error (risk of windup) |
| `K_D` | 0.17 | Pointing | Higher = more damping (slower convergence) |

### Inner Loop (PI Current Controller)

**File:** `config.h`

| Gain | Value | Description |
|---|---|---|
| `PIL_KP` | 5.0 | Proportional gain |
| `PIL_KI` | 1500.0 | Integral gain |
| `PIL_T` | 0.001 | Sample time (1kHz) |
| `PIL_MAX_VOLTAGE` | 3.3V | Output clamp (datasheet typical supply) |

### HITL Runtime Calibration Override

In HITL mode, host now sends runtime calibration in every packet:

- `max_voltage_mV` (uint16): firmware runtime voltage clamp
- `dipole_strength_milli` (uint16): firmware runtime dipole conversion basis

Host CLI behavior:

- `--max-voltage` and `--dipole-strength` are optional overrides.
- If omitted, host uses hardcoded defaults `3.3V` and `2.88 Am^2/A`.
- Overrides are automatic; no additional debug flag is required.

Host/Firmware packet compatibility:

- Input packet format: `<H3f3f3f4f5fBHH`
- Input packet size: `79` bytes
- Output payload format (after sync bytes): `<3fB9hB>`
- Output payload size: `32` bytes (`34` including sync header)
- Host and firmware versions must match this schema (`telemetry_flags` version bits).

### Telemetry Quantization Scales

**File:** `cube_sat_nucleo/Application/Algorithms/Inc/config.h`

| Constant | Value | Meaning |
|---|---|---|
| `TELEMETRY_PACKET_VERSION` | `1` | Host/Firmware output packet version |
| `M_CMD_FULL_SCALE_AM2` | `8.0` | Q15 full-scale for commanded dipole telemetry |
| `TAU_FULL_SCALE_NM` | `0.05` | Q15 full-scale for raw/projected torque telemetry |

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
