---
trigger: always_on
---

# GEMINI.md — CubeSat ADCS Project Context

## Project Overview

Flight Software (FSW) and HITL simulation for a CubeSat ADCS using magnetorquer-only actuation on **STM32 NUCLEO-L476RG**.

**Hardware:** STM32L476RG (Cortex-M4), BNO085 IMU (SPI), DRV8833 H-Bridge (PWM), ADT7420 Temp (I2C), Magnetorquer coils (R=28Ω, L=12–25mH).

## Current Status

### Completed
- [x] **Inner Loop**: PI current controller at 1kHz — tracks ±95mA.
- [x] **Outer Loop**: Full ADCS state machine with 3 modes (Detumble → Pointing, with safety fallback).
- [x] **Physics Engine**: RK4 integrator with RL sub-stepping, tilted dipole B-field model, inertial/body frame separation.
- [x] **HITL Framework**: Binary protocol over UART (115200 baud) with runtime gain tuning.
- [x] **Telemetry**: Teleplot integration + 3D cube visualization.
- [x] **Control Law Verification**: B-dot detumble, PID pointing with 180° singularity recovery.

### In Progress / Known Issues
- [ ] Refine B-dot polarity — positive `k_bdot` produces spin-up, negative produces damping. Root cause: possible coordinate mapping inversion between physics engine and firmware.
- [ ] Spin Stabilization mode — basic implementation exists, needs tuning.
- [ ] Sensor noise injection for robustness testing.

## Software Architecture

### Firmware (`cube_sat_nucleo/`)

| Layer | Key Files | Purpose |
|---|---|---|
| **Outer Loop** | `outer_loop_control.c`, `config.h` | ADCS mode selection + control laws |
| **Inner Loop** | `inner_loop_control.c`, `pi_controller.c` | 1kHz current tracking |
| **Drivers** | `hbridge.c`, `current_sensor.c`, `imu_bno085.c` | Hardware abstraction |
| **Core** | `main.c`, `main.h` | HAL init, UART dispatch, packet structs |
| **Math** | `math_lib.c` | `Vec3_Cross`, `Vec3_Norm`, quaternion ops |

### Physics Engine (`physics_engine/`)

| File | Purpose |
|---|---|
| `physics.py` | `SatellitePhysics` class: dynamics, kinematics, RK4, RL circuits |
| `simulation_host.py` | Main loop, CLI args, scenario setup, telemetry dispatch |
| `comms.py` | Binary packed serial protocol (sync header `0xB562`) |
| `telemetry.py` | Teleplot UDP + 3D visualization sender |

### Control Modes

```
┌──────────┐   ω < 0.02 rad/s   ┌──────────┐
│ DETUMBLE │ ─────────────────→ │ POINTING │
│ (B-dot)  │ ←───────────────── │  (PID)   │
└──────────┘   ω > 0.15 rad/s   └──────────┘
       │
       ↓ (manual)
┌──────────────┐
│ SPIN STABLE  │
│ (Y-axis spin)│
└──────────────┘
```

### Key Configuration (`config.h`)

| Parameter | Value | Description |
|---|---|---|
| `K_BDOT` | 200000.0 | B-dot gain (ensures MTQ saturation) |
| `K_P` | 0.100 | Pointing proportional gain |
| `K_I` | 0.0001 | Pointing integral gain |
| `K_D` | 0.100 | Pointing derivative gain |
| `MTQ_DIPOLE_TO_AMP` | 1/5.76 | Dipole (Am²) → current (A) conversion |
| `DETUMBLE_OMEGA_THRESH` | 0.02 rad/s | Transition to Pointing |
| `POINTING_OMEGA_THRESH` | 0.15 rad/s | Fallback to Detumble |

### HITL Protocol

**Input (Host → Firmware):** 70 bytes packed, `__attribute__((packed))`
```
[Header:u16][I_xyz:3f][ω_xyz:3f][B_xyz:3f][q_wxyz:4f][K_bdot:f][Kp:f][Ki:f][Kd:f][dt:f][flags:u8][pad:3x]
```

**Output (Firmware → Host):** 16 bytes packed
```
[Header:u16][Vx:f][Vy:f][Vz:f][mode:u8][pad:3x]
```

## References

- **COTS Implementation of Magnetorquer-Only CubeSat Spin Stabilization** (Cornell University, Umansky-Castro et al.)
- **BNO085** CEVA/Hillcrest Labs 9-Axis SiP
- **DRV8833** TI Dual H-Bridge Motor Driver
- **STM32L476RG** Reference Manual
