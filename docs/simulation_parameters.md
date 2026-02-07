# Simulation Parameters for Stress Testing

This document outlines the key parameters in the Python Physics Engine that control the satellite's physical behavior. Adjust these values to simulate different CubeSat configurations or stress-test the control algorithms.

## 1. Satellite Mass Properties (Inertia)
**File:** `physics_engine/physics.py`
**Class:** `SatellitePhysics`

These parameters determine how hard it is to spin the satellite (Angular Acceleration for a given Torque).

| Parameter | Current Value | Description | Effect of Increasing |
| :--- | :--- | :--- | :--- |
| `self.I` (Matrix) | `diag([0.00833, 0.00833, 0.00333])` | Inertia Tensor for a 2U CubeSat (kg·m²). | **Slower Spin-Up**: Satellite reacts more sluggishly to torque. |

**Tuning Guide:**
*   **Asymmetric 2U:** The current values assume a uniform 2U shape ($I_{xx} = I_{yy} > I_{zz}$).
*   **1U Config:** Change to `[0.002, 0.002, 0.002]`.
*   **Deployables:** Increase $I_{xx}$ or $I_{yy}$ if solar panels deploy, increasing the moment of inertia.

## 2. Magnetorquer Efficiency (Torque Generation)
**File:** `physics_engine/simulation_host.py`
**Loop:** `main()` (inside response handling)

These parameters control how much magnetic torque is generated for a given current.

| Parameter | Current Value | Description | Effect of Increasing |
| :--- | :--- | :--- | :--- |
| `m_eff` | `0.4` (Am²/A) | **Effective Dipole Moment per Amp**. Represents $N \times A$ (Turns × Area). | **Stronger Actuation**: Higher torque for the same current. Faster spin-up/detumble. |
| `R_coil` | `25.0` (Ohms) | Electrical resistance of the coil. | **Lower Current**: Reduces the current (and thus torque) for a given voltage command. |
| `L_coil` | `0.1` (Henry) | Inductance of the coil. | **Slower Response**: Current takes longer to reach steady state after a voltage step. |

**Equation:**
$$ \tau = \mathbf{m} \times \mathbf{B} $$
$$ \mathbf{m} = \text{Current} \times \text{m\_eff} $$
$$ \text{Current} \approx V / R_{\text{coil}} $$

## 3. Environmental Conditions
**File:** `physics_engine/physics.py`
**Method:** `get_magnetic_field(t)`

| Parameter | Current Value | Description | Effect of Increasing |
| :--- | :--- | :--- | :--- |
| `50e-6` | $50 \mu T$ | Magnitude of Earth's Magnetic Field. | **Higher Torque**: Both Disturbances and Control Torques become stronger. |
| Orbit Period | `5400s` (90 min) | Rate of B-Field rotation. | **Faster B-Field Change**: Controller must adapt faster to changing field direction. |

## 4. Simulation Timing
**File:** `physics_engine/simulation_host.py`

| Parameter | Current Value | Description | Effect of Increasing |
| :--- | :--- | :--- | :--- |
| `DT` | `0.01` (10ms) | Physics Step Size. | **Lower Accuracy**: Physics integration may become unstable if too large. |

## Stress Testing Scenarios

1.  **Weak Actuator Test:**
    *   Reduce `m_eff` to `0.1`.
    *   **Goal:** Verify B-Dot controller can still detumble, albeit slowly.

2.  **High Inertia Test:**
    *   Double `self.I` values.
    *   **Goal:** Simulate a heavy deployable payload.

3.  **Low Friction/Damping Test:**
    *   Currently, there is **NO** environmental drag torque.
    *   **Goal:** The satellite should conserve energy perfectly (Angular velocity constant constant when `target_current = 0`).

4.  **Sensor Noise (Future):**
    *   Add `random.gauss(0, sigma)` to `B_body` in `simulation_host.py` before sending to firmware.
    *   **Goal:** Test robustness of the B-Dot derivative filter.
