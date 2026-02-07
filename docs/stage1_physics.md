# Implementation Stage 1: Physics Engine (Python)

**Goal:** Create a simulation environment that models Rigid Body Dynamics to test the flight software before deployment.

#### A. Mathematical Model

The engine must integrate **Euler's Rotation Equations** and Kinematics.

1. **Dynamics (Torque $\rightarrow$ Angular Acceleration):**

   $$ \dot{\omega} = I^{-1} (\tau_{ext} - \omega \times (I \omega)) $$

   * $I$: Inertia Tensor ($3 \times 3$ matrix).
   * $\tau_{ext}$ (Torque from coils).


2. **Kinematics (Angular Velocity $\rightarrow$ Quaternion Rate):**

   $$ \dot{q} = \frac{1}{2} \Omega q $$

   * Where $\Omega$ is the skew-symmetric matrix of angular velocity.


3. **Environmental Model:**
   * **Magnetic Field ($B$):** Use a simple Dipole Model or IGRF library (`ppigrf`).
   * **Drag/Disturbance:** Add random noise to $\dot{\omega}$.



#### B. Data Handler & Communication

* **Packet Structure (PC $\rightarrow$ STM32):**
  `struct { float q[4]; float w[3]; float B[3]; }`
* **Packet Structure (STM32 $\rightarrow$ PC):**
  `struct { float dipole_cmd[3]; }`

#### C. Deliverables

* `simulation_host.py`: Main loop running RK4 integration (10ms steps).
* `physics.py`: Class implementing the equations of motion.
* `comms.py`: Serial port handler for binary data transfer.
