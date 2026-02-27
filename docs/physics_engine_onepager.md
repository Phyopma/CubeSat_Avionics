# ADCS Physics Engine One-Pager (Meeting Handout)

Last updated: 2026-02-11

## 1) Two-component architecture

### A. Environment model (Python host + physics)
- Simulated state: quaternion `q`, body rate `w`, coil current `i`.
- Equations:
  - `q_dot = 0.5 * Omega(w) * q`
  - `i_dot = (V_cmd - iR)/L`
  - `tau = (i * dipole_strength) x B_body`
  - `w_dot = I^-1 * (tau - w x (Iw))`
- Outputs sent as simulated sensors to firmware:
  - gyro = `w`
  - mag = `B_body`
  - attitude = `q`
  - current = `i`

### B. Magnetorquer control/actuation (STM32 firmware)
- Outer loop computes dipole request `m_cmd` (B-dot / Kane / Pointing).
- Main converts dipole to target current:
  - `I_target = m_cmd * MTQ_DIPOLE_TO_AMP`
- Inner loop computes H-bridge command voltage:
  - PI mode: `V_cmd = PI(I_target, I_measured)`
  - Open-loop debug: `V_cmd = I_target * R`
- Firmware returns `V_cmd` to host, and host applies that in the RL plant next step.

## 2) Why host uses `V_cmd` (not `I_target`) and whether this is OK

Your understanding is correct:
- MTQ torque is proportional to **current**, not voltage.
- Voltage matters because it drives current through RL dynamics.

Why `V_cmd` feedback is physically correct in this HITL loop:
1. Firmware is the real controller/actuator driver producing `V_cmd`.
2. Plant physics computes resulting current using `R, L`:
   - `i_dot = (V_cmd - iR)/L`
3. Torque is then computed from current:
   - `tau = (i * dipole_strength) x B`

So the chain is:
- `I_target -> controller -> V_cmd -> RL -> i_actual -> torque`

This is good architecture because it preserves actuator dynamics/saturation.

When it becomes concerning:
1. If modeled `R/L` are wrong vs hardware.
2. If `PIL_MAX_VOLTAGE` (firmware) differs from host `--max-voltage`.
3. If `dipole_strength` and `MTQ_DIPOLE_TO_AMP` are not reciprocal.
4. If coil resistance changes with temperature but model is fixed.

## 3) Critical constants to align across teams

| Category | Variable | Current value | Why critical |
|---|---|---:|---|
| Structure | `Px, Py, Pz` | `0.011867, 0.012575, 0.012867 kg*m^2` | Principal moments loaded from W26 constants |
| Structure | `I_body` | `R*diag(P)*R^T` | Full body-frame tensor used by dynamics |
| Power | `PIL_MAX_VOLTAGE` / host `--max-voltage` | `3.3 V` | Sets max current slew and authority |
| Power | `R` / `Lxy` / `Lz` | `28 ohm / 25mH / 12mH` | Determines current dynamics + power draw |
| Actuator | `dipole_strength` | `2.88 A*m^2/A` | Current-to-dipole conversion |
| Actuator | `MTQ_DIPOLE_TO_AMP` | `1/2.88 A/(A*m^2)` | Dipole-to-current command mapping |
| Control | `K_BDOT` | `400000` | Detumble behavior and transition readiness |
| Control | `K_P, K_I, K_D` | `0.0012, 3e-05, 0.17` | Pointing stability/recovery |

## 4) Current tuning status (iteration summary)

1. Earlier baseline (`2026-02-10`) had weak long-run pointing robustness.
2. Added pointing robustness logic (projection-aware torque, anti-windup, scheduling, transition hardening).
3. Current default set from latest iteration:
   - `K_BDOT=400000`
   - `K_P=0.0012`
   - `K_I=3e-05`
   - `K_D=0.17`
4. Latest 600s baseline matrix (`2026-02-11`) ended all 4 cases in `POINTING`; `PT_B` is no longer catastrophic.

## 5) Key metrics to monitor in stress tests

- `Sat` (voltage saturation ratio)
- `ProjLoss` (how much requested torque is uncommandable due to magnetic geometry)
- `IntClamp` (integrator clamped/frozen fraction)
- `Transitions` (mode chatter)
- `Tsettle` (time to settle)

## 6) Quick recommendations before team sync

1. Host now auto-overrides firmware voltage/dipole in HITL every packet; keep host defaults at `3.3/2.88` unless intentionally testing different authority.
2. Confirm W26 principal-axis orientation against latest mass layout and sign conventions.
3. If power team expects large thermal drift, add temperature-aware `R(T)` in model.
4. Keep evaluating `PT_B` with 600s runs; prioritize reducing `ProjLoss`/`IntClamp`.

## 7) HITL packet compatibility note

- Host input packet format is `<H3f3f3f4f5fBHH>` (79 bytes).
- Tail fields are:
  - `max_voltage_mV`
  - `dipole_strength_milli`
- Host output payload format (after sync bytes) is `<3fB9hB>` (32 bytes payload, 34 bytes including header).
- Output includes:
  - commanded dipole (`m_cmd_q15`)
  - raw desired torque (`tau_raw_q15`)
  - projected torque (`tau_proj_q15`)
  - `telemetry_flags` (version + saturation bits)
- Host and firmware must be updated together for this schema.

## 8) Clickable references

- Full technical brief: [/Users/phyopyae/eecs159/docs/physics_engine_system_brief.md](/Users/phyopyae/eecs159/docs/physics_engine_system_brief.md)
- Physics model: [/Users/phyopyae/eecs159/physics_engine/physics.py](/Users/phyopyae/eecs159/physics_engine/physics.py)
- Host loop + CLI + telemetry: [/Users/phyopyae/eecs159/physics_engine/simulation_host.py](/Users/phyopyae/eecs159/physics_engine/simulation_host.py)
- Firmware constants: [/Users/phyopyae/eecs159/cube_sat_nucleo/Application/Algorithms/Inc/config.h](/Users/phyopyae/eecs159/cube_sat_nucleo/Application/Algorithms/Inc/config.h)
- Outer loop control: [/Users/phyopyae/eecs159/cube_sat_nucleo/Application/Algorithms/Src/outer_loop_control.c](/Users/phyopyae/eecs159/cube_sat_nucleo/Application/Algorithms/Src/outer_loop_control.c)
- Inner loop control: [/Users/phyopyae/eecs159/cube_sat_nucleo/Application/Algorithms/Src/inner_loop_control.c](/Users/phyopyae/eecs159/cube_sat_nucleo/Application/Algorithms/Src/inner_loop_control.c)
- Main control wiring: [/Users/phyopyae/eecs159/cube_sat_nucleo/Core/Src/main.c](/Users/phyopyae/eecs159/cube_sat_nucleo/Core/Src/main.c)
- Latest baseline matrix (2026-02-11): [/Users/phyopyae/eecs159/physics_engine/results/baseline_matrix_20260211_150732.txt](/Users/phyopyae/eecs159/physics_engine/results/baseline_matrix_20260211_150732.txt)
- Baseline stress JSON (2026-02-11): [/Users/phyopyae/eecs159/physics_engine/results/stress600_baseline_20260211_134859.json](/Users/phyopyae/eecs159/physics_engine/results/stress600_baseline_20260211_134859.json)

Research papers:
- [Magnetic detumbling foundations (Avanzini & Giulietti)](https://doi.org/10.2514/1.53074)
- [B-dot revisitation (TAC)](https://doi.org/10.1109/TAC.2015.2458293)
- [Kane damper origin](https://doi.org/10.2514/3.3683)
- [Psiaki/Stengel magnetic ADCS lineage index](https://www.princeton.edu/~stengel/MAE342.html)
