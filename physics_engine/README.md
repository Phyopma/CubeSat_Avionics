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
```

When `--duration` is set, the host prints:
- `FINAL_STATE: W=... | Err=...`
- `FINAL_METRICS: Mode=... Dwell=... ForcedDwell=... Sat=... Transitions=... Tsettle=...`

## Protocol

Packed binary structs over UART at 115200 baud. Sync header: `0xB5 0x62`.

### Host → Firmware (70 bytes)

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
| Padding | uint8 | 3 | Alignment |

### Firmware → Host (16 bytes)

| Field | Type | Count | Description |
|---|---|---|---|
| Header | uint16 | 1 | `0x62B5` |
| Voltage X | float32 | 1 | Commanded voltage (V) |
| Voltage Y | float32 | 1 | Commanded voltage (V) |
| Voltage Z | float32 | 1 | Commanded voltage (V) |
| ADCS Mode | uint8 | 1 | 0=Idle, 1=Detumble, 2=Spin, 3=Pointing |
| Padding | uint8 | 3 | Alignment |

## File Structure

| File | Description |
|---|---|
| `physics.py` | Satellite dynamics, RK4 integrator with RL sub-stepping |
| `simulation_host.py` | Main loop, CLI, scenario setup |
| `comms.py` | Serial protocol (pack/unpack, sync) |
| `telemetry.py` | Teleplot UDP + 3D cube visualization |
