# Outer Loop Parameter Tuning & Physics Consistency Plan

## Summary
Fine-tune outer loop ADCS parameters, analyze physics consistency, and update documentation.

## Critical Finding: Dipole Factor Discrepancy

| Source | Value | Calculation |
|--------|-------|-------------|
| **MTQ Datasheet** | **2.88 Am²/A** | 0.34 Am² @ 3.3V, R=28Ω → I=0.118A → 0.34/0.118 |
| **Code (physics.py:76)** | **5.76 Am²/A** | Comment: "Double authority" |

The code uses exactly 2× the datasheet value. This affects all gain tuning.

## Documentation Discrepancies Found

| Parameter | Docs | Code | Action |
|-----------|------|------|--------|
| DETUMBLE_OMEGA_THRESH | 0.02 rad/s | 0.03 rad/s | Update docs |
| K_BDOT | 200,000 (positive) | -200,000 | Clarify sign in docs |
| Default DT | 0.1s | 0.01s (code) | Update docs |

---

## Implementation Plan

### Phase 1: Fix Physics Parameters

**1. Dipole Factor (2.88 Am²/A from datasheet):**

- [physics.py:76](physics_engine/physics.py#L76): Change `m_actual = i * 5.76` → `i * 2.88`
- [config.h:44](cube_sat_nucleo/Application/Algorithms/Inc/config.h#L44): Change `MTQ_DIPOLE_TO_AMP (1.0f / 5.76f)` → `(1.0f / 2.88f)`

**2. Voltage Clamp (3.3V from datasheet typical supply):**

- [physics.py:88](physics_engine/physics.py#L88): Change `np.clip(v_command, -5.0, 5.0)` → `np.clip(v_command, -3.3, 3.3)`
- [inner_loop_control.h:21](cube_sat_nucleo/Application/Algorithms/Inc/inner_loop_control.h#L21): Change `MAX_OUTPUT_VOLTAGE 5.0f` → `3.3f`
- [config.h:11](cube_sat_nucleo/Application/Algorithms/Inc/config.h#L11): Change `PIL_MAX_VOLTAGE 5.0f` → `3.3f`

**Impact of 3.3V clamp:**
- Max current: I = 3.3V / 28Ω = 0.118A (matches datasheet condition)
- Max dipole: m = 0.118A × 2.88 = 0.34 Am² (matches datasheet)

### Phase 2: Run Parameter Tuning

```bash
cd /Users/phyopyae/eecs159.tmp/physics_engine
source .venv/bin/activate
uv run tune_params.py
```

**Enhanced tuning grid (modify tune_params.py):**

With ~3× reduced torque authority, gains should be ~3× higher.

**Phase 2a: Coarse Sweep (find region)**

Detumble (K_BDOT):
- Coarse: [-200000, -400000, -600000, -800000, -1000000, -1500000, -2000000]

Pointing (PID):
- K_P coarse: [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80]
- K_I coarse: [0.0001, 0.0002, 0.0005, 0.001, 0.002]
- K_D coarse: [0.10, 0.20, 0.30, 0.40, 0.50]

**Phase 2b: Fine Sweep (refine best region)**

After identifying best coarse region, run fine sweep around it:

Detumble (K_BDOT) fine:
- Step size: 50000 around best coarse value
- Example if best coarse = -600000: [-500000, -550000, -600000, -650000, -700000]

Pointing (PID) fine:
- K_P fine: step 0.02 around best (e.g., [0.26, 0.28, 0.30, 0.32, 0.34])
- K_I fine: step 0.0001 around best (e.g., [0.0003, 0.0004, 0.0005, 0.0006, 0.0007])
- K_D fine: step 0.02 around best (e.g., [0.26, 0.28, 0.30, 0.32, 0.34])

**Phase 2c: Ultra-Fine Sweep (perfect params)**

Final refinement with smallest steps:
- K_P: step 0.005
- K_I: step 0.00002
- K_D: step 0.005
- K_BDOT: step 10000

### Phase 2.5: Fix Code Issues

**1. Quaternion Convention Consistency**

User switched quaternion convention in physics.py. Need to update:
- [physics.py:125-131](physics_engine/physics.py#L125): `_quat_to_dcm()` - verify matches new Omega matrix convention
- [outer_loop_control.c:84-89](cube_sat_nucleo/Application/Algorithms/Src/outer_loop_control.c#L84): Pointing math DCM extraction - verify matches physics convention

The Omega matrix in physics.py (lines 62-67) and _quat_to_dcm() must use consistent convention for attitude to propagate correctly.

**2. B-Dot Alignment Kick Fix**

[outer_loop_control.c:60](cube_sat_nucleo/Application/Algorithms/Src/outer_loop_control.c#L60): Current `m_cmd.x += 0.1f` fails if B≈X-axis.

Fix: Use axis perpendicular to B:
```c
// Find axis perpendicular to B for kick
vec3_t kick_axis;
if (fabsf(B.x) < fabsf(B.z)) {
    kick_axis = (vec3_t){1.0f, 0.0f, 0.0f}; // X if B not aligned with X
} else {
    kick_axis = (vec3_t){0.0f, 0.0f, 1.0f}; // Z if B near X-axis
}
vec3_t kick = Vec3_ScalarMult(kick_axis, 0.1f);
m_cmd = Vec3_Add(m_cmd, kick);
```

**3. Compilation Fix (sim_input.dt)**

[main.c:208](cube_sat_nucleo/Core/Src/main.c#L208): `sim_input.dt` used outside SIMULATION_MODE.

Fix: Add default dt for non-simulation mode:
```c
#ifdef SIMULATION_MODE
    OuterLoop_Update(&adcs_in, &adcs_out, sim_input.dt);
#else
    OuterLoop_Update(&adcs_in, &adcs_out, 0.01f); // 100Hz default
#endif
```

**4. Test Script Fix**

[physics_verify.py:19](physics_engine/physics_verify.py#L19): `get_magnetic_field()` → `get_magnetic_field_inertial()`

### Phase 3: Update Documentation

1. [simulation_parameters.md](docs/simulation_parameters.md):
   - Line 73: `DETUMBLE_OMEGA_THRESH` → 0.03 rad/s
   - Add dipole factor section explaining 2.88 vs 5.76
   - Clarify K_BDOT sign convention

2. [stage2_firmware.md](docs/stage2_firmware.md):
   - Update threshold table to match config.h

### Phase 4: Validation

```bash
# Test detumble with tuned gains
uv run simulation_host.py --scenario detumble --duration 120 --debug

# Test pointing with tuned gains
uv run simulation_host.py --scenario pointing --duration 180 --debug

# Verify damping (should see angular velocity decrease)
```

Success criteria:
- Detumble: ω < 0.03 rad/s within 60s
- Pointing: Error < 15° within 90s

---

## Files to Modify

| File | Changes |
|------|---------|
| [physics.py](physics_engine/physics.py) | Dipole 5.76→2.88, voltage 5.0→3.3, verify _quat_to_dcm() convention |
| [inner_loop_control.h](cube_sat_nucleo/Application/Algorithms/Inc/inner_loop_control.h) | MAX_OUTPUT_VOLTAGE 5.0→3.3 |
| [config.h](cube_sat_nucleo/Application/Algorithms/Inc/config.h) | PIL_MAX_VOLTAGE 5.0→3.3, MTQ_DIPOLE_TO_AMP 1/5.76→1/2.88, tuned gains |
| [outer_loop_control.c](cube_sat_nucleo/Application/Algorithms/Src/outer_loop_control.c) | Fix B-dot kick logic, verify pointing DCM extraction |
| [main.c](cube_sat_nucleo/Core/Src/main.c) | Fix sim_input.dt compilation issue |
| [physics_verify.py](physics_engine/physics_verify.py) | Fix get_magnetic_field() → get_magnetic_field_inertial() |
| [tune_params.py](physics_engine/tune_params.py) | Expand parameter grid (coarse→fine→ultra-fine) |
| [simulation_parameters.md](docs/simulation_parameters.md) | Fix threshold, update voltage/dipole specs |
| [stage2_firmware.md](docs/stage2_firmware.md) | Update to match code |

---

## User Decision: Use Realistic Hardware Parameters

**Dipole Factor:** 2.88 Am²/A (datasheet) instead of 5.76
**Supply Voltage:** 3.3V (datasheet typical) instead of 5.0V

**Combined effect on torque authority:**
- Old: I_max = 5.0V/28Ω = 0.179A, m_max = 0.179 × 5.76 = 1.03 Am²
- New: I_max = 3.3V/28Ω = 0.118A, m_max = 0.118 × 2.88 = 0.34 Am²
- Reduction factor: ~3× less torque authority

Gains will need significant increase (~3×) to compensate:
- K_BDOT: -200000 → start testing at -600000
- K_P/K_D: May need ~3× increase (test 0.3)
