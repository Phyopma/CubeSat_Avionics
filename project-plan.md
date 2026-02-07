---
trigger: always_on
---

# GEMINI.md - CubeSat Avionics Context

## Project Overview
This repository contains the Flight Software (FSW) for a CubeSat avionics system based on the **STM32 NUCLEO-L476RG**. The project focuses on Attitude Determination and Control Systems (ADCS) using magnetic actuation.

**Key Hardware:**
* **MCU:** STM32L476RG (ARM Cortex-M4)
* **IMU:** BNO085 (9-DOF Absolute Orientation) via SPI/I2C
* **Actuator Driver:** DRV8833 Dual H-Bridge (PWM controlled)
* **Temperature Sensor:** PmodTMP2 (ADT7420) via I2C
* **Magnetorquers:** Inductive coils for magnetic dipole generation

## Software Architecture
The codebase is organized into layers to separate hardware dependencies from control logic:

1.  **Core (HAL Layer):** STM32CubeMX generated code (`Core/Src`, `Core/Inc`) handling low-level peripherals (Timers, SPI, I2C, UART).
2.  **Drivers (`Application/Drivers`):**
    * `hbridge.c`: Low-level PWM abstraction for the DRV8833.
    * `imu_bno085.c`: Interface for retrieving Quaternion, Mag, and Gyro data.
    * `current_sensor.c`: Feedback mechanism for the inner control loop.
3.  **Algorithms (`Application/Algorithms`):**
    * `inner_loop_control.c`: PI Controller ensuring coil current matches commanded targets.
    * `pi_controller.c`: Generic PI implementation.

## Development Status
* **Completed:**
    * Basic peripheral drivers (I2C, SPI, UART).
    * Inner Loop Control: Accurate current driving into magnetorquers using PI control.
    * H-Bridge PWM generation and direction control.
* **In Progress:**
    * Outer Loop Control (B-Dot / Pointing Algorithms).
    * Hardware-in-the-Loop (HITL) Simulation framework.

---

## Roadmap & To-Do List

### Phase 1: Simulation Framework (HITL)
The immediate goal is to close the feedback loop virtually, as the dev board cannot physically rotate in response to torque.
- [ ] **A. Mathematical Model (Physics Engine)**
    - [ ] Implement `physics.py` Class Structure.
    - [ ] Implement Rigid Body Dynamics ($\dot{\omega} = I^{-1} (\tau - \omega \times I\omega)$).
    - [ ] Implement Kinematics (Quaternion Integration $\dot{q} = 0.5 \Omega q$).
    - [ ] Implement Environmental Model (Dipole Magnetic Field).
- [ ] **B. Data Flow & Communications**
    - [ ] Define Binary Packet Structure (PC <-> STM32).
    - [ ] Implement `comms.py` for Serial I/O (handling synchronization).
    - [ ] Create `simulation_host.py` Main Loop (RK4 Integration @ 10ms steps).
- [ ] **C. Firmware Preparation (STM32)**
    - [ ] Clean up `main.c`: Disable ad-hoc sensor tests.
    - [ ] Verify `SIMULATION_MODE` hooks in `main.c` and `imu_bno085.c`.
    - [ ] Ensure `InnerLoop` takes external torque/current commands.
- [ ] **D. Verification**
    - [ ] **Unit Test:** Verify Energy Conservation in `physics.py` (Torque-free motion).
    - [ ] **Integration Test:** Loopback Comms Test (Send Data -> Receive Reply).
    - [ ] **System Test:** Visual correctness in Teleplot (Simulated tumbling).

### Phase 2: ADCS Algorithm Development
- [ ] **Implement B-Dot Controller:**
    - Calculate the derivative of the magnetic field vector.
    - Generate dipole commands to oppose angular velocity (`m = -k * B_dot`).
- [ ] **Implement Pointing Controller (Spin Stabilization):**
    - Reference: "COTS Implementation of Magnetorquer-Only CubeSat Spin Stabilization".
    - Implement feedback linearization or simple PD control on the spin axis.

### Phase 3: Mission Logic
- [ ] **State Machine:**
    - Define modes: `DETUMBLE`, `ACQUISITION`, `NORMAL_OPS`, `SAFE_MODE`.
    - Implement transitions based on angular rate thresholds (e.g., if `omega < 0.1 rad/s` -> transition to `NORMAL_OPS`).

---

## References & Knowledge Base

**Primary Control Algorithm Reference:**
> **Title:** COTS Implementation of Magnetorquer-Only CubeSat Spin Stabilization
> **Authors:** Joshua Umansky-Castro et al. (Cornell University)
> **Context:** Describes using a physics model to compute external magnetic field and angular velocity in place of sensor inputs to verify ACS algorithms on stationary hardware.

**Datasheets:**
* **BNO085:** CEVA/Hillcrest Labs 9-Axis SiP (Reference for SPI readout and SHTP protocol).
* **STM32H745ZI / STM32L476RG:** Reference for Timer/PWM and DMA configurations.
* **DRV8833:** TI Dual H-Bridge Motor Driver (Reference for decay modes and current limits).
