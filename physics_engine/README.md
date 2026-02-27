# Physics Engine — HITL Simulation Host

Python-based physics engine for Hardware-in-the-Loop simulation of the CubeSat ADCS. Communicates with STM32 firmware over binary UART protocol.

## Prerequisites

- [uv](https://github.com/astral-sh/uv) for dependency management
- Python 3.10+
- STM32 Nucleo-L476RG connected via USB

## Setup

```bash
uv sync
```

## Usage

### Detumble Scenario
```bash
# Default: high initial tumble → firmware reduces angular velocity
uv run simulation_host.py --scenario detumble --initial_omega 1.0 1.0 1.0

# With debug output (prints ω, B-field, dipole moment)
uv run simulation_host.py --scenario detumble --debug

# With custom gains
uv run simulation_host.py --scenario detumble --kbdot 500000.0

# Force a specific STM32 outer-loop mode (bypass state machine)
uv run simulation_host.py --scenario detumble --force-mode detumble
```

### Pointing Scenario
```bash
# 180° flip recovery test (starts anti-parallel to target)
uv run simulation_host.py --scenario pointing --initial_omega 0.05 0.05 0.05

# Custom PID gains
uv run simulation_host.py --scenario pointing --kp 0.2 --kd 0.15
```

### Timing Options
```bash
# Fast mode (default): 0.1s steps, ~10 min per orbit
uv run simulation_host.py --scenario detumble

# Realtime: 1.0s steps, 90 min per orbit
uv run simulation_host.py --scenario detumble --realtime

# Custom step size
uv run simulation_host.py --scenario detumble --dt 0.05
```

### Other Flags
```bash
# Open loop (bypass PI controller: V = I·R)
uv run simulation_host.py --open-loop

# Request deterministic controller state reset at start (default on)
uv run simulation_host.py --reset-controller

# Disable start-of-run reset request
uv run simulation_host.py --no-reset-controller

# Finite duration (auto-terminates, prints final state)
uv run simulation_host.py --scenario detumble --duration 120.0

# Quiet mode (no console output)
uv run simulation_host.py --quiet

# Use a specific structural constants source file
uv run simulation_host.py --structural-config ./config/structural_constants_w26.json
```

When `--duration` is set, the host prints:
- `FINAL_STATE: W=... | Err=...`
- `FINAL_METRICS: Mode=... Dwell=... ForcedDwell=... Sat=... Transitions=... Tsettle=...`

## Protocol

Packed binary structs over UART at 115200 baud. Sync header: `0xB5 0x62`.

### Host → Firmware (79 bytes total)

| Field | Type | Count | Description |
|---|---|---|---|
| Header | uint16 | 1 | `0x62B5` (appears as `B5 62` on wire) |
| Currents | float32 | 3 | Simulated coil currents (A) |
| Gyro | float32 | 3 | Angular velocity (rad/s) |
| Mag | float32 | 3 | Body-frame B-field (T) |
| Quaternion | float32 | 4 | Attitude (qw, qx, qy, qz) |
| K_bdot | float32 | 1 | Runtime B-dot gain |
| K_p | float32 | 1 | Runtime pointing P gain |
| K_i | float32 | 1 | Runtime pointing I gain |
| K_d | float32 | 1 | Runtime pointing D gain |
| dt | float32 | 1 | Simulation step size (s) |
| Debug flags | uint8 | 1 | Bit 0: open-loop, Bits 1-2: force mode (`00` auto, `01` detumble, `10` spin, `11` pointing), Bit 3: controller reset request |
| max_voltage_mV | uint16 | 1 | Runtime inner-loop voltage limit override |
| dipole_strength_milli | uint16 | 1 | Runtime dipole conversion override (`Am²/A * 1000`) |

### Firmware → Host v2 (34 bytes total, 32-byte payload after sync)

| Field | Type | Count | Description |
|---|---|---|---|
| Header | uint16 | 1 | `0x62B5` |
| Voltage X | float32 | 1 | Commanded voltage (V) |
| Voltage Y | float32 | 1 | Commanded voltage (V) |
| Voltage Z | float32 | 1 | Commanded voltage (V) |
| ADCS Mode | uint8 | 1 | bits0-1 mode, bit2 integral clamp flag, bits3-7 projection-loss quantized |
| m_cmd_q15 | int16 | 3 | Dipole command vector normalized by `M_CMD_FULL_SCALE_AM2` |
| tau_raw_q15 | int16 | 3 | Raw desired torque vector normalized by `TAU_FULL_SCALE_NM` |
| tau_proj_q15 | int16 | 3 | Projected commandable torque vector normalized by `TAU_FULL_SCALE_NM` |
| telemetry_flags | uint8 | 1 | bits0-3 packet version, bit4/5/6 saturation flags for m/raw/proj |

Python unpack format for payload after sync bytes:

`<3fB9hB`

## Structural Constants

The host loads structural/orbit constants from:

- `physics_engine/config/structural_constants_w26.json`

Loaded values include:
- Orbit period, inclination, RAAN, semimajor axis
- Center of mass (for traceability)
- Principal moments and principal-axis directions

`physics.py` builds a full body-frame inertia tensor using normalized/orthogonalized principal axes:

`I_body = R * diag(Px, Py, Pz) * R^T`

## File Structure

| File | Description |
|---|---|
| `config/structural_constants_w26.json` | W26-derived structural and orbit constants |
| `constants_loader.py` | Structural constants validation + inertia tensor construction |
| `physics.py` | Satellite dynamics, RK4 integrator with RL sub-stepping |
| `simulation_host.py` | Main loop, CLI, scenario setup |
| `comms.py` | Serial protocol (pack/unpack, sync) |
| `telemetry.py` | Teleplot UDP + 3D cube visualization |
