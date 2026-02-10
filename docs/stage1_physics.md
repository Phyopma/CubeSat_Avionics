# Implementation Stage 1: Physics Engine (Python)

**Goal:** Simulate rigid body dynamics, RL circuit magnetorquer current, and an orbital magnetic field model for HITL testing.

**Status:** ✅ Complete

## Mathematical Model

### 1. Rigid Body Dynamics

$$\dot{\omega} = I^{-1} (\tau_{mag} - \omega \times I\omega)$$

- $I$: Diagonal inertia tensor for a 2U CubeSat
- $\tau_{mag} = m \times B_{body}$: Torque from magnetorquer dipole crossed with local B-field
- $m = I_{coil} \times 2.88$: Effective magnetic dipole moment (Am²)

### 2. Quaternion Kinematics

$$\dot{q} = \frac{1}{2} \Omega({\omega}) \cdot q$$

Where $\Omega$ is the 4×4 skew-symmetric matrix built from angular velocity components. Quaternion normalization is enforced every sub-step to prevent drift.

### 3. RL Circuit Model

$$\frac{dI}{dt} = \frac{V_{cmd} - I \cdot R}{L}$$

Per-axis inductance values (X/Y: 25mH, Z: 12mH) with R = 28Ω. RK4 sub-stepping ensures stability (τ = L/R ≈ 0.4–0.9ms, sub_dt ≤ 0.5ms).

### 4. Magnetic Field Model (Tilted Dipole)

The inertial-frame B-field is computed from a tilted dipole model:

$$B_{inertial} = B_0 \left( 3(\hat{m} \cdot \hat{r})\hat{r} - \hat{m} \right)$$

- $B_0 = 50\mu T$, orbit period = 5400s (90min LEO)
- Inclination = 51.6° (ISS-like), dipole tilt = 11.7°
- Body-frame conversion: $B_{body} = R_{body}^T \cdot B_{inertial}$ (dynamically rotated inside each RK4 sub-step)

## Integration Method

**RK4 with automatic sub-stepping:**

1. Outer step size set by CLI (default 0.1s for fast mode)
2. RL circuit requires sub_dt ≤ 0.5ms for stability → auto-calculated `n_sub = ceil(dt / 0.0005)`
3. Quaternion normalized every sub-step
4. NaN/Inf safety checks with automatic recovery

## State Vector

```python
state = [q_w, q_x, q_y, q_z, ω_x, ω_y, ω_z, i_x, i_y, i_z]
#         0     1    2    3    4    5    6    7   8    9
```

## Key Design Decisions

1. **Inertial/Body frame separation**: B-field is computed in inertial frame once per outer step, then rotated to body frame inside each RK4 sub-step using the current quaternion. This significantly improves accuracy at large time steps.
2. **Sub-stepping**: Rather than shrinking the global step size, only the stiff RL dynamics are sub-stepped. Attitude dynamics are stable at 0.1s steps.
3. **Voltage clamping**: Applied before integration to match real H-bridge limits (±3.3V).

## Deliverables

| File | Description |
|---|---|
| `physics.py` | `SatellitePhysics` class — dynamics, kinematics, RK4, B-field model |
| `simulation_host.py` | Main HITL loop, CLI interface, telemetry dispatch |
| `comms.py` | Binary serial protocol with sync header (`0xB562`) |
| `telemetry.py` | Teleplot UDP + 3D quaternion visualization |
