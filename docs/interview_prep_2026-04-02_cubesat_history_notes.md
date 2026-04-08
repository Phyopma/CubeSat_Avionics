# EECS159 CubeSat Project History Notes

Date: 2026-04-02
Repo: `/Users/phyopyae/eecs159`
Purpose: reconstruct the project timeline, branch roles, implementation choices, debugging patterns, and code areas most useful for interview prep.

## 1. High-level repo shape

The repo has two tightly coupled halves:

- Firmware on STM32 Nucleo in [`/Users/phyopyae/eecs159/cube_sat_nucleo`](/Users/phyopyae/eecs159/cube_sat_nucleo)
- Physics/HITL host in [`/Users/phyopyae/eecs159/physics_engine`](/Users/phyopyae/eecs159/physics_engine)

The strongest interview narrative is not "I wrote one embedded module." It is:

- I brought up hardware drivers and low-level comms on STM32.
- I built a HITL loop so real firmware could drive a simulated plant.
- I debugged timing, packet sync, and numerical stability issues across firmware and Python.
- I then layered ADCS logic on top: B-dot detumble, virtual damper spin stabilization, and quaternion-based pointing.
- I finally refactored the runtime from a super-loop into a FreeRTOS task model and split the demos into sensor-only and simulation-only branches.

## 2. Branch map and what each branch means

### Current branch families

- `main`
  - Latest bare-metal baseline.
  - Also aliased by `fix/verbose-teleplot`.
  - Contains the later non-RTOS ADCS/tuning line up through `Plan HITL telemetry upgrade`.

- `feat/hitl-simulation`
  - First branch for basic HITL simulation framework.
  - Main unique commits:
    - `5c6384f` `feat: implement HITL simulation framework`
    - `0babc0b` `refactor: move python host and update docs`

- `feat/hitl-waveform-gen`
  - Extends early HITL with waveform generation and most of the serious protocol/debugging work.
  - Main unique commits:
    - `dca9005` waveform generation
    - `cb06a07` UDP Teleplot
    - `ce19344` ping-pong sync
    - `7daa8dd` buffer flush + revert to 1 kHz
    - `ed8e2e2` inductance stability fix
    - `de02b9f` header sync
    - `4628693` endian fix + infinite-loop protection

- `feat/control-algo-physic-engine-stage-1`
  - Stage 1 physics/control groundwork.
  - Ends at `45bfbeb` `feat: complete stage 1 and prepare for stage 2 telemetry`.

- `feat/stage-2-init`
  - Stage 2 integration checkpoint after tuned gains/HITL refactor.
  - Tip: `e2fafae`.

- `fix/adcs-tuning-refinement`
  - Small refinement branch around open-loop verification and consistency checks.
  - Tip: `cb56303`.

- `feat/adcs-control-refine`
  - Later debug-oriented branch after stage 2.
  - Commits:
    - `2634643` `WIP: refine adcs control and fix build error`
    - `17ff2e7` `fix: move ClampF to global scope to resolve invalid storage class error`
    - `d9682ff` `debug: added run logs`

- `feat/rtos-init`
  - Main RTOS migration branch from the bare-metal line.
  - Carries the full port of physics/HITL/ADCS into FreeRTOS.

- `demo-simulation`
  - Final RTOS simulation-only demo branch.
  - Branch-selected runtime, not flag-selected.

- `demo-sensor`
  - Final RTOS sensor-only demo branch.
  - Branch-selected runtime, not flag-selected.

### Older exploratory branches

- `feature/thermocouple_nucleo`
  - Surviving branch tip points at shared docs fix `d6722d4`.
  - The real thermocouple implementation history is preserved mostly in early `main` commits, not in unique surviving branch-only commits.

- `feature/Hbridge`
  - Same situation as thermocouple branch tip.
  - H-bridge evolution is visible more clearly in the mainline and RTOS branches than from branch tip metadata.

- `feature/IMU`
  - Preserves IMU-focused bring-up work plus a later merge from main.
  - Unique commit:
    - `d37fe61` `added flag for testing`
  - Tip:
    - `765f8bc` merge from `uci/main`.

- `feature/att_control`
  - Early conceptual control-system branch.
  - Unique commit:
    - `d156601` `refined typedef for consistent communication for control algo`
  - Important because it shows your earlier architecture thinking before the later ADCS implementation stabilized in `Application/Algorithms`.

## 3. Project timeline

## Phase A: initial board bring-up and simple sensing

- `2025-10-20 e24b279` `Initial commit`
- `2025-10-20 6c4c8d5` `thermocouple firmware done; serial output needs work`
- `2025-10-21 9fb1350` `serial output fixed`
- `2025-10-21 7c2c947` `thermocouple testing done`

What this says about your workflow:

- You started with single-sensor bring-up.
- You used serial output as the first validation tool.
- You treated "firmware works" and "observability works" as separate milestones.

## Phase B: IMU bring-up and protocol debugging

- `2025-10-30 d9a49d6` `IMU service failed: SPI continuation flag on`
- `2025-11-03 4286d87` `IMU service failed: stuck at waiting INT pin reset after write`
- `2025-11-06 d9c08f4` `implemented and working basic communication with IMU`
- `2025-11-21 14b0c3c` `added individual sensor reading`
- `2025-11-29 bec8105` `added retry sensor activation cmd for IMU`
- `2026-01-18 c780560` `removed duplicated ACK waiting logic for RV reading (IMU)`

What this says:

- You debugged at the transport/protocol layer, not just application logic.
- The commit messages show a strong failure-driven workflow:
  - identify exact failure mode
  - change low-level protocol handling
  - validate with working communication
  - add retries / simplify duplicated waits later

Representative code evidence:

- IMU driver parses SHTP packets with continuation handling in [`imu_bno085.c`](/Users/phyopyae/eecs159/cube_sat_nucleo/Application/Drivers/Src/imu_bno085.c).
- The driver now has explicit report-length classification and instrumentation for first-seen report IDs.
- The older `feature/IMU` branch still shows the comment around the missing timestamp report fix, which is a good interview example of protocol-framing bugs.

## Phase C: inner-loop control and test flags

- `2025-11-21 d9b7f42` `added template for inner closed pi control and implemented open loop for testing; restructured main with control flags for specific testing`
- `2025-11-29 eb99ad3` `tunned pi inner loop`
- `2025-11-29 d37fe61` `added flag for testing`

What this says:

- You did not jump directly into full ADCS.
- You first established current control and test harnessing.
- You used feature flags in `main.c` to isolate hardware paths during bring-up.

This is visible in the older `feature/IMU` `main.c`, where compile-time flags gate sensor and control paths.

## Phase D: documentation and architecture capture

- `2026-01-23 ea06f51` `added README, ctrl, dataflow and architecture diagrams`
- `2026-01-23 bb30056` `Create README.md`
- `2026-01-23 5ce65c7` `Update documentation links for project structure`

This is useful for interviews because it shows you moved from pure coding into system communication:

- architecture diagrams
- control/data flow docs
- repo documentation

## Phase E: HITL framework creation

- `2026-01-28 5c6384f` `feat: implement HITL simulation framework`
- `2026-01-28 0babc0b` `refactor: move python host and update docs`

Core result:

- Real STM32 firmware drives a simulated plant on the host.
- The host and firmware exchange binary packets over serial.
- The host simulates attitude, rates, current, and magnetic field.

Files that define this architecture:

- [`simulation_host.py`](/Users/phyopyae/eecs159/physics_engine/simulation_host.py)
- [`physics.py`](/Users/phyopyae/eecs159/physics_engine/physics.py)
- [`inner_loop_control.c`](/Users/phyopyae/eecs159/cube_sat_nucleo/Application/Algorithms/Src/inner_loop_control.c)
- [`main.c`](/Users/phyopyae/eecs159/cube_sat_nucleo/Core/Src/main.c)

Interview angle:

- You built a closed loop across embedded firmware and a software plant to validate control behavior before full hardware integration.

## Phase F: waveform generation and multi-step debugging of HITL

- `2026-01-29 dca9005` `feat: add waveform generation for HITL`
- `2026-01-30 cb06a07` `feat: enable udp telemetry for Teleplot`
- `2026-01-30 d807dbf` `chore: update teleplot address to teleplot.fr`
- `2026-01-30 c6da95a` `chore: enable timestamped telemetry for better plotting`
- `2026-01-30 ce19344` `fix: implement ping-pong sync for HITL to solve latency`
- `2026-01-30 7daa8dd` `fix: revert firmware loop to 1kHz, impl python buffer flush`
- `2026-01-30 ed8e2e2` `fix: increase simulated inductance for numerical stability`
- `2026-01-30 de02b9f` `fix: implement header-based serial sync (0xB562) to prevent garbage reads`
- `2026-01-30 4628693` `fix: endianness of sync header and infinite loop protection`

This is one of the strongest interview stories in the repo.

You can tell it as a sequence:

1. Build initial HITL.
2. Add better excitation and telemetry.
3. See stale/laggy behavior.
4. Fix host/firmware synchronization.
5. See unstable numerical behavior.
6. Fix simulation model constants and time-step stability.
7. See garbage values from packet misalignment.
8. Add packet framing.
9. Hit an endianness issue.
10. Fix byte order and add timeout protection.

The walkthrough doc in [`docs/features/hitl_framework.md`](/Users/phyopyae/eecs159/docs/features/hitl_framework.md) already preserves this reasoning sequence cleanly.

## Phase G: Stage 1 physics model formalization

- `2026-02-07 d27e6c6` `feat: update specs to 2U & add physics docs`
- `2026-02-07 45bfbeb` `feat: complete stage 1 and prepare for stage 2 telemetry`

This phase tightened the physics story:

- rigid body dynamics
- quaternion kinematics
- RL current dynamics
- orbital magnetic field model

Relevant docs:

- [`stage1_physics.md`](/Users/phyopyae/eecs159/docs/stage1_physics.md)
- [`physics_engine_onepager.md`](/Users/phyopyae/eecs159/docs/physics_engine_onepager.md)

## Phase H: Stage 2 ADCS and tuning pipeline

- `2026-02-09 922ee0e` `feat(adcs): complete hitl-only refactor and apply optimal tuned gains`
- `2026-02-09 cb56303` `feat(adcs): add open-loop debug flag and verify physics consistency`
- `2026-02-09 e73119b` `feat/adcs-pid: Implemented PID controller for pointing mode, added integral term with anti-windup, upgraded physics engine to Tilted Dipole model, and verified gains via automated tuning.`
- `2026-02-10 adc7382` `Update CubeSat tuning flow`
- `2026-02-10 568f47e` `Add pointing stall thresholds`
- `2026-02-11 2634643` `WIP: refine adcs control and fix build error`
- `2026-02-11 17ff2e7` `fix: move ClampF to global scope to resolve invalid storage class error`
- `2026-02-19 d9682ff` `debug: added run logs`

What changed technically:

- Added firmware-side `config.h` constants.
- Added `outer_loop_control.c` and `math_lib.c`.
- Host/firmware packet schema matured.
- Physics engine moved to a tilted-dipole magnetic model.
- Pointing mode gained integral action with anti-windup.
- Tuning became more systematic with optimizer/tuning scripts.

This is where the project stops being "sensor + current loop" and becomes a full ADCS implementation.

## Phase I: RTOS migration and demo split

- `2026-02-19 ca16460` `feat: transition to free-rtos`
- `2026-02-27 1d255b9` `feat: port config.h constants from main`
- `2026-02-27 ef3934b` `feat: port math_lib (vec3/quat) from main`
- `2026-02-27 3adcc88` `feat: port outer loop control (ADCS) from main`
- `2026-02-27 73fe0fc` `feat: upgrade hbridge to 3-axis API`
- `2026-02-27 188111f` `feat: upgrade SimPacket to v2 (3-axis + ADCS telemetry)`
- `2026-02-27 71555af` `feat: upgrade inner loop to 3-axis with config.h constants`
- `2026-02-27 8c30050` `feat: restructure app_runtime as pure RTOS with ADCS task`
- `2026-02-27 4a9ecd7` `feat: clean main.c — remove legacy flags, port sync-byte UART RX`
- `2026-02-27 36b63f4` `refactor: use RTOS critical sections in serial_log_dma`
- `2026-02-27 7c8157e` `feat: port physics engine from main (HITL, telemetry, tuning)`
- `2026-02-27 34fc583` `docs: port documentation from main`
- `2026-02-27 5342004` `feat: configure FreeRTOS via CubeMX and fix main.c integration`
- `2026-03-01 cb0551c` `[feat : hardware sensor-log checkpoint with configurable print rate]`
- `2026-03-01 316f9b8` `[fix : harden RTOS sensor init logging and BNO085 report handling]`
- `2026-03-01 2a0c21c` `[feat : add teleplot port and usb modem args to simulation host]`
- `2026-03-01 db98482` `[feat : add teleplot port and usb modem args to simulation host]`

This is the other major interview story:

- You recognized the limitations of a super-loop architecture.
- You migrated to FreeRTOS.
- You extracted the runtime into `app_runtime.c`.
- You separated simulation and sensor demos by branch, not by compile flag.

## 4. How you tend to add features

Across the repo, your pattern is:

1. Add a narrow capability.
2. Add instrumentation/telemetry around it.
3. Add configuration flags or runtime parameters.
4. Refactor once the path is proven.

Examples:

- Sensors:
  - thermocouple working
  - serial output fixed
  - test done

- IMU:
  - failing protocol commits first
  - basic communication commit
  - individual readings
  - retries
  - later ACK simplification

- HITL:
  - initial framework
  - waveform generation
  - telemetry
  - sync fixes
  - packet framing

- ADCS:
  - B-dot and control groundwork
  - stage-1 physics
  - stage-2 refactor
  - tuning refinements

- RTOS:
  - initial transition
  - port algorithms
  - restructure runtime
  - harden sensor/IMU handling
  - split demo branches

This is a good interview talking point because it shows iterative engineering instead of one-shot coding.

## 5. How you debug

Your debugging style is visible directly in the commit messages and code:

### A. Name the exact failure mode

Examples:

- `IMU service failed: SPI continuation flag on`
- `stuck at waiting INT pin reset after write`
- `solve latency`
- `increase simulated inductance for numerical stability`
- `prevent garbage reads`
- `resolve invalid storage class error`

That style is strong in interviews because it sounds like systems debugging, not guessing.

### B. Debug across boundaries, not inside one file

Best example: HITL sync bugs

- Firmware serial transmit path
- Host serial read path
- packet framing
- loop frequency mismatch
- plotting/telemetry evidence

You did not treat firmware and host as separate silos.

### C. Add observability before or alongside fixes

Examples:

- serial output fixes early in project
- Teleplot UDP telemetry
- timestamped telemetry
- waiting-for-host logs in RTOS logger task
- `debug: added run logs`

### D. Use fallback/safety logic after finding failure modes

Examples in code:

- Packet timeout in host read path to avoid infinite hanging.
- `SIM_PACKET_TIMEOUT_MS` zero-command fail-safe in RTOS ADCS task.
- quaternion renormalization and NaN/Inf protection in physics engine.
- runtime clamp on voltage/dipole overrides.
- integral clamp and projection-loss handling in pointing mode.

## 6. Deep code notes

## Firmware runtime wiring

### Current main entry point

- [`main.c`](/Users/phyopyae/eecs159/cube_sat_nucleo/Core/Src/main.c)

Main observations:

- Peripherals are initialized in `main()`.
- `AppRuntime_Init()` and `AppRuntime_Start()` take over.
- UART receive interrupt starts immediately for sync-byte state-machine receive.
- `while(1)` is effectively dead after scheduler start.

This is a clean post-refactor embedded structure:

- `main.c` becomes board bootstrap
- `app_runtime.c` owns behavior

### RTOS runtime

- [`app_runtime.c`](/Users/phyopyae/eecs159/cube_sat_nucleo/Core/Src/app_runtime.c)

Important details:

- `ADCSTask` is event-driven off packet notifications, not simple polling.
- On host packet timeout, target current is forced to zero.
- Host can inject:
  - force mode
  - reset request
  - dynamic gains
  - runtime voltage clamp
  - runtime dipole conversion

That is a strong interview point because it shows you designed the RTOS boundary around control safety and tuning flexibility, not just task scheduling.

## Outer-loop ADCS logic

- [`outer_loop_control.c`](/Users/phyopyae/eecs159/cube_sat_nucleo/Application/Algorithms/Src/outer_loop_control.c)

Key behaviors:

- Default mode is `DETUMBLE`.
- State machine transitions into `POINTING` only after low-rate hold time.
- Falls back from `POINTING` when unstable or stalled.
- Telemetry byte packs mode, integral clamp state, and projection-loss ratio without growing packet size.

### B-dot implementation

Notable details:

- Uses gyro-estimated `cross(w, B)` rather than finite-difference B-dot.
- Includes a kick when spin is high but estimated `B_dot` is too small, which handles the "aligned with B field" dead zone.

This is better than a textbook-only implementation because it includes a practical escape mechanism.

### Pointing implementation

Notable details:

- Quaternion-to-body target conversion is explicit.
- Handles 180-degree singularity by injecting a small X-axis kick.
- Uses gain scheduling based on pointing error.
- Uses integral leak and conditional integration rather than always integrating.
- Projects requested torque onto the plane orthogonal to the magnetic field.
- Measures projection loss and blends toward B-dot damping when geometry is poor.

This is one of the best code areas to discuss in interview because it shows:

- geometric reasoning
- actuator constraint awareness
- anti-windup thinking
- fallback logic under poor controllability

## Inner-loop control

- [`inner_loop_control.c`](/Users/phyopyae/eecs159/cube_sat_nucleo/Application/Algorithms/Src/inner_loop_control.c)

Important details:

- PI controller runs on all 3 axes.
- Open-loop debug mode is still available.
- Output packet includes:
  - voltage command
  - mode byte
  - quantized dipole request
  - quantized raw/projected torque
  - telemetry flags with saturation bits

This is useful in interviews because you can explain that the inner loop is not just actuator control; it is also a telemetry bridge that exposes controller internals for HITL debugging.

## Host/physics side

### Simulation host

- [`simulation_host.py`](/Users/phyopyae/eecs159/physics_engine/simulation_host.py)

Important details:

- Reads compile-time firmware constants by parsing `config.h`.
- Clamps host-provided runtime overrides to firmware-supported ranges.
- Supports mode forcing, open-loop testing, reset requests, and CLI-controlled gains.
- Accepts either explicit serial port or USB modem suffix.
- Supports configurable Teleplot UDP port.

This is an unusually strong piece for interviews because it shows you built tooling around the firmware rather than treating firmware as isolated.

### Physics engine

- [`physics.py`](/Users/phyopyae/eecs159/physics_engine/physics.py)

Important details:

- Uses a full inertia tensor loaded from structural constants.
- Simulates RL coil dynamics and attitude dynamics together.
- Uses RK4 with sub-stepping because RL dynamics are stiffer than attitude dynamics.
- Renormalizes quaternion every sub-step.
- Clips values for protocol safety.

This shows you were reasoning about both controls and numerical methods, not just writing packet glue.

### Serial protocol

- [`comms.py`](/Users/phyopyae/eecs159/physics_engine/comms.py)

Important details:

- Explicit struct formats for host->firmware and firmware->host.
- Sync bytes `0xB5 0x62`.
- Telemetry version checking to catch host/firmware mismatches early.
- Host input buffer flush before writes to prioritize fresh responses.

This directly supports the debugging story from the January 30 fixes.

## 7. Early control-system architecture branch

The older `feature/att_control` branch is worth mentioning even though the final code moved elsewhere.

It contains:

- `acs_t` system handle
- mode enums
- sensor/control output structs
- a `magnetorquer_t` abstraction
- control constants for Kane damper style logic

What that means:

- Before the final `Application/Algorithms` implementation stabilized, you were already thinking in full ACS subsystem terms:
  - sensor fusion inputs
  - actuator abstraction
  - mode management
  - calibration
  - power/fault/status structure

This can be used in interviews to show architectural intent even if the final implementation changed shape.

## 8. What seems most aligned with the Western Digital firmware interview

The strongest evidence in this repo for firmware/systems alignment is:

- SPI/I2C/UART bring-up and debugging on STM32.
- Interrupt-driven UART receive state machine with sync-byte protocol.
- 1 kHz control loop thinking and later RTOS task design.
- PI current control on 3 axes.
- Binary protocol design and versioning.
- Safety/fallback logic for stale packets and saturation.
- Hardware/software co-design through a Python HITL harness.

The less central but still good supporting material:

- thermocouple and current sensor drivers
- documentation and architecture diagrams
- physics-model/tuning tooling

## 9. Good interview themes to build later

Based on this history, the best future prep topics are:

- "Walk me through a hard bug you debugged."
  - Use HITL latency + packet sync + endian story.

- "Tell me about an embedded system you designed."
  - Use STM32 firmware + host simulation + ADCS pipeline.

- "How do you validate firmware without full hardware availability?"
  - Use the HITL setup.

- "Tell me about a refactor."
  - Use super-loop to FreeRTOS migration.

- "Tell me about controls or actuator constraints."
  - Use torque projection, anti-windup, and B-dot fallback.

## 10. Caveats from repo archaeology

- Some branch tips, especially `feature/Hbridge` and `feature/thermocouple_nucleo`, no longer preserve unique branch-only code beyond shared commits.
- The strongest surviving evidence comes from:
  - commit messages
  - merge graph
  - current codebase
  - older branch snapshots such as `feature/IMU` and `feature/att_control`
- The mainline after February 2026 captures most of the mature ADCS/control logic better than the older feature branches.
