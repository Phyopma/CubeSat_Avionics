# Western Digital Firmware Interview Cheat Sheet

## 30-second intro

I built STM32 firmware for a CubeSat magnetorquer control system and paired it with a Python hardware-in-the-loop simulator. My work included sensor bring-up over SPI and I2C, a binary UART protocol, inner current-loop control, higher-level ADCS logic, and later a FreeRTOS runtime refactor. The project is the best example of how I work in firmware: isolate problems, add observability, validate end to end, and iterate until the system is stable.

## Best stories

- HITL latency bug: stale serial packets because firmware and host loops ran at different effective rates.
- Packet framing bug: garbage float values caused by byte misalignment, fixed with sync header and endian correction.
- Physics stability bug: simulation instability traced to RL time constant vs integration step.
- IMU bring-up: SPI/SHTP debugging, retries, and report parsing fixes.
- RTOS refactor: moved from `main.c` super-loop to task-based runtime in `app_runtime.c`.

## Best technical themes

- STM32L476 firmware
- SPI, I2C, UART, DMA, timers
- interrupt-driven serial receive
- binary protocol design
- PI current control
- B-dot and pointing control
- actuator constraints
- HITL validation
- FreeRTOS migration

## Best behavioral themes

- I use data to resolve disagreements.
- I add telemetry and docs so other people can reason about the system too.
- I prefer isolating one variable at a time over making broad speculative changes.
- I treat debugging as a system problem, not just a code problem.

## If they ask “why Western Digital?”

I want to work on software that sits close to the hardware, where timing, protocols, reliability, and system constraints matter. That’s the kind of work I enjoyed most in my CubeSat project, and it’s the environment where I think I’ll grow fastest.

## If they ask “biggest weakness?”

Early on I sometimes went too deep technically before making the system picture obvious to everyone else. I’ve improved that by documenting interfaces, using telemetry and diagrams, and making my reasoning easier for the team to follow.

## If they ask “tell me about conflict”

I handle conflict by narrowing it to a concrete technical question and bringing evidence. In the CubeSat project, a realistic disagreement was how much to trust closed-loop ADCS behavior before the inner current-control path and actuator calibration were fully validated. I kept open-loop available as a diagnostic mode, added telemetry around target current, measured current, voltage, and saturation, and used HITL runs to separate controller issues from actuator-path issues. In system projects, disagreement is often really uncertainty about where the problem lives. Logs, telemetry, and small experiments usually resolve that faster than debate.

## Team context to mention

- UCI CubeSat is a larger student organization with Executive, Advisory, Avionics, Communications, Power, Structures, and Systems.
- Your role is best framed as Avionics software.
- Your own docs show structure, power, and ADCS/control coordination points.
- Best phrasing: “I worked on the Avionics software side, but our work had to line up with power and actuator limits, structure and inertia assumptions, and system integration.”

## Files to remember

- [`outer_loop_control.c`](/Users/phyopyae/eecs159/cube_sat_nucleo/Application/Algorithms/Src/outer_loop_control.c)
- [`inner_loop_control.c`](/Users/phyopyae/eecs159/cube_sat_nucleo/Application/Algorithms/Src/inner_loop_control.c)
- [`app_runtime.c`](/Users/phyopyae/eecs159/cube_sat_nucleo/Core/Src/app_runtime.c)
- [`simulation_host.py`](/Users/phyopyae/eecs159/physics_engine/simulation_host.py)
- [`physics.py`](/Users/phyopyae/eecs159/physics_engine/physics.py)
- [`imu_bno085.c`](/Users/phyopyae/eecs159/cube_sat_nucleo/Application/Drivers/Src/imu_bno085.c)

## Commits to remember

- `5c6384f` initial HITL framework
- `ce19344` ping-pong sync
- `7daa8dd` fresh-buffer fix + 1 kHz correction
- `de02b9f` sync header
- `4628693` endianness fix
- `e73119b` pointing PID + tilted dipole update
- `ca16460` FreeRTOS transition
- `8c30050` RTOS runtime restructure
