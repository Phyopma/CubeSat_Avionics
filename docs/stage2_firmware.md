# Implementation Stage 2: Control Algorithms (STM32 Firmware)

**Goal:** Implement the "Brain" of the ADCS. This module takes sensor data and outputs current commands. It must switch between three distinct modes.

#### Data Structures

The control system requires a state machine struct:

```c
typedef enum {
    CTRL_MODE_DETUMBLE,      // B-Dot
    CTRL_MODE_SPIN_STABLE,   // Kane Damper
    CTRL_MODE_POINTING       // Active Inertial Pointing
} adcs_mode_t;

typedef struct {
    adcs_mode_t mode;
    float k_bdot;            // Gain for Detumbling
    float c_damp;            // Kane: Damping coefficient
    float I_d;               // Kane: Virtual Inertia
    float w_d[1];            // Kane: Virtual Damper State
    float q_target[2];       // Pointing: Target Attitude
} adcs_control_t;
```

#### Mode A: Detumbling (B-Dot)

* **Trigger:** Mission start or high angular rates ($|\omega| > 20^\circ/s$).
* **Math:**

   $$M = -k \dot{B}$$

   * *Implementation Note:* Since we don't measure $\dot{B}$ directly, use discrete derivative: $\dot{B} \approx \frac{B_n - B_{n-1}}{\Delta t}$.


* **Output:** Maximizes magnetic drag to stop rotation.

#### Mode B: Spin Stabilization (Virtual Kane Damper)

* **Trigger:** Angular rates low, need to establish Gyroscopic Stiffness.
* **Math (Virtual Integrator):**

   $$\dot{w}_d = I_d^{-1} (\tau_{mag} \cdot \hat{z} - c_{damp} w_d)$$

   * *Update:* Integrate this differential equation using RK4 or Trapezoidal method every loop cycle.


* **Math (Torque Request):**

   $$\tau_{req} = -c_{damp} w_d \hat{z}$$


* **Math (Dipole Projection):**

   $$M = \frac{1}{B^2} (\tau_{req} \times B)$$



#### Mode C: Inertial Pointing (PD Controller)

* **Trigger:** Spin is stable, need to point Z-axis at Sun/Ground.
* **Math (Error Quaternion):**

   $$q_{err} = q_{target}^{-1} * q_{current}$$


* **Math (Control Law):**

   $$\tau_{req} = -K_p q_{err}.v - K_d \omega$$

   * *Note:* $K_d$ provides damping, $K_p$ provides the "spring" force to align axes.



#### D. Deliverables

* `outer_loop_control.c`: State machine and math implementations.
* `math_lib.c`: Helper functions for Vector Cross Products, Quaternion Math, and Matrix multiplication.
* `config.h`: Gain definitions ($K_p, K_d, c_{damp}$).
