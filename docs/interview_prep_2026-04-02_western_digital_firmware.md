# Western Digital - Software Development Engineer (Firmware) Interview Prep

Candidate: Phyo Pyae Aung
Date: 2026-04-02
Primary source project: [`EECS159 CubeSat repo`](/Users/phyopyae/eecs159)
Supporting repo archaeology: [`CubeSat history notes`](/Users/phyopyae/eecs159/docs/interview_prep_2026-04-02_cubesat_history_notes.md)

## 1. What the interview is likely to optimize for

Based on recent public interview reports and the role title, expect a mix of:

- project deep dive on the work you personally owned
- embedded/firmware fundamentals
- C and systems questions
- debugging questions
- collaboration and communication
- some light algorithm/problem-solving or puzzle-style questioning

For you, the strongest strategy is to anchor almost everything to the CubeSat project and keep the answers concrete:

- what problem existed
- what hardware/software boundary existed
- what you changed
- how you validated it
- what tradeoff you made

Do not try to sound like a storage-firmware engineer already. Sound like a strong early-career embedded engineer who has already done real firmware, protocol, control-loop, debugging, and HITL work.

## 2. Your strongest project positioning

### Short version

I built firmware for an STM32-based CubeSat magnetorquer control system and paired it with a Python hardware-in-the-loop simulation so I could validate control logic, serial protocols, and system behavior before relying on full flight hardware. Over time I brought up sensors, implemented and tuned the current-control loop, built the HITL framework, added higher-level ADCS logic, and later refactored the firmware runtime from a bare-metal super-loop to a FreeRTOS task-based architecture.

### Why this aligns with firmware

This project lets you demonstrate:

- embedded C on STM32
- SPI, I2C, UART
- interrupt-driven packet handling
- control-loop implementation
- binary protocol design
- RTOS migration
- real debugging across hardware and host-software boundaries
- validation through telemetry and HITL

## 3. Your 60-second opening answer

### Script

I’m a computer science student, but the project I’d highlight most for firmware is my EECS159 CubeSat magnetorquer control system. I worked on STM32 firmware for sensor bring-up, serial communication, current-loop control, and later higher-level ADCS logic. One of the biggest things I built was a hardware-in-the-loop setup where the real firmware ran on the Nucleo board while a Python physics engine simulated the satellite’s dynamics and sent sensor data over a binary serial protocol. That let me debug issues like packet misalignment, stale data, timing mismatches, and control instability in a much more realistic way than unit tests alone. Later I refactored the runtime into FreeRTOS tasks and split the project into simulation and sensor demo branches. That project is the best example of how I work on embedded systems: build observability first, isolate failure modes, validate changes end to end, and iterate until the system is stable.

## 4. Your 2-minute project walkthrough

### Script

The project was a CubeSat magnetorquer control system running on an STM32L476 Nucleo board. At a low level, I had to bring up drivers for components like the BNO085 IMU over SPI, a current sensor over I2C, and telemetry over UART. On top of that, I implemented the inner current loop for the magnetorquers, then added a higher-level ADCS controller with detumble and pointing behavior.

One of the most useful parts of the project was the hardware-in-the-loop framework. Instead of testing only in firmware or only in a desktop simulator, I had the real firmware talk to a Python host that simulated rigid body dynamics, the magnetic field, and RL current dynamics. The host sent simulated gyro, magnetometer, quaternion, and current data to the firmware, and the firmware sent back commanded voltages and control telemetry.

That setup exposed a lot of real system issues. For example, I debugged stale serial packets caused by a loop-rate mismatch, garbage values caused by packet byte misalignment, and numerical instability in the plant model because the simulation step was too large relative to the RL time constant. I fixed those with buffer flushing, explicit sync headers, endianness corrections, and better physical parameters.

Later I refactored the firmware from a bare-metal super-loop into a FreeRTOS runtime with separate tasks for ADCS, sensor sampling, IMU servicing, and logging. That made the structure cleaner and closer to how a more mature firmware system should be organized.

## 5. Story bank

Use these stories repeatedly. Most technical and behavioral answers should come from this list.

### Story A: HITL latency and stale-data debugging

Use for:

- debugging
- system thinking
- working across boundaries
- validating assumptions

Situation:

- The firmware loop was much faster than the Python host loop.
- The host was reading old buffered packets, so the system looked laggy and unresponsive.

Task:

- Find why the HITL response felt delayed and make the data path reflect the freshest state.

Action:

- Compared firmware and host loop timing.
- Realized the OS serial buffer was accumulating stale packets.
- Added host-side input buffer flushing before reads.
- Reworked synchronization behavior so the host and firmware exchanged fresher data.
- Verified the effect through telemetry plots.

Result:

- Latency dropped dramatically and the closed-loop response became much more immediate and believable.

Why it is strong:

- Shows you debugged the system, not just one function.

### Story B: Packet framing and endianness bug

Use for:

- tricky bug
- binary protocol design
- low-level reasoning

Situation:

- The HITL loop occasionally produced absurd floating-point values.

Task:

- Determine whether the issue was in control logic, the simulator, or the communication layer.

Action:

- Realized raw serial reads had no strong framing, so partial-byte alignment could corrupt floats.
- Added a 2-byte sync header.
- Updated the host to scan for that header before unpacking payloads.
- Then discovered the STM32 was little-endian, so the header bytes were arriving reversed from the host expectation.
- Corrected the on-wire header value and added timeout protection in the host scan loop.

Result:

- Eliminated garbage packet reads and removed a hang condition at the same time.

Why it is strong:

- Shows protocol awareness, embedded data-format reasoning, and disciplined isolation of root cause.

### Story C: Numerical instability in the simulator

Use for:

- engineering judgment
- modeling
- validation

Situation:

- The simulated current could oscillate wildly or blow up even with small inputs.

Task:

- Determine whether the controller was unstable or whether the plant model was unstable.

Action:

- Looked at the RL time constant versus simulation step size.
- Recognized the Euler-style setup was unstable because the step size was too large relative to the coil dynamics.
- Increased the simulated inductance to restore stability in the chosen integration regime.
- Later used RK4 sub-stepping in the physics model for a more robust implementation.

Result:

- The current response became smooth and physically plausible, which made subsequent controller tuning meaningful.

Why it is strong:

- Shows you understand when a bug is mathematical, not purely software.

### Story D: IMU bring-up and protocol debugging

Use for:

- hardware bring-up
- persistence
- debugging vendor protocols

Situation:

- The BNO085 IMU initially failed during service and initialization, with issues around SPI packet continuation and startup sequencing.

Task:

- Get stable sensor communication and usable reports into the system.

Action:

- Debugged the SPI/SHTP packet flow.
- Fixed continuation handling and report parsing assumptions.
- Added retry logic around sensor activation.
- Simplified duplicated ACK waiting logic once the data path was understood better.

Result:

- Reached stable IMU communication and individual sensor readings, which unblocked the rest of the control stack.

Why it is strong:

- Very aligned with firmware bring-up work.

### Story E: Refactor from super-loop to FreeRTOS

Use for:

- architectural refactor
- code quality
- scalability
- teamwork/process

Situation:

- The original system behavior lived largely in `main.c` with multiple feature flags and a super-loop structure.

Task:

- Make the runtime more modular, scalable, and appropriate for concurrent sensor/control/logging work.

Action:

- Created `app_runtime.c` to own runtime behavior.
- Added RTOS tasks for ADCS, sensor sampling, IMU service, telemetry, and logging.
- Preserved safety by zeroing commands on packet timeout.
- Reduced branch ambiguity by separating simulation and sensor demos into different branches rather than compile-flag combinations.

Result:

- The runtime became easier to reason about and closer to production-style embedded structure.

Why it is strong:

- Shows you can improve architecture, not just add features.

## 6. Technical interview questions with tailored answers

## Q1. Tell me about a firmware project you’re proud of.

### Answer

The best example is my EECS159 CubeSat project. I worked on firmware for an STM32-based magnetorquer control system and built a Python hardware-in-the-loop environment around it. On the firmware side I brought up sensors, implemented current-loop control, built a binary UART protocol, and later added higher-level ADCS logic and a FreeRTOS runtime. On the host side I built the simulation layer that modeled attitude dynamics, magnetic field behavior, and coil current dynamics. What I’m most proud of is that I didn’t just write firmware in isolation. I built a system where I could observe, debug, and validate the firmware against a realistic plant, which made my debugging and tuning much stronger.

## Q2. What microcontroller and peripherals did you use?

### Answer

I used the STM32L476RG Nucleo board. The main peripherals I worked with were SPI for the BNO085 IMU, I2C for sensors like the current sensor and temperature sensor, UART for telemetry and the HITL serial protocol, timers for the control loop and PWM-related actuation, DMA for serial logging, and later FreeRTOS task scheduling around the runtime.

## Q3. Describe a difficult embedded bug you solved.

### Answer

One difficult bug was in the hardware-in-the-loop serial protocol. I was seeing occasional nonsense floating-point values in the host, which at first could have been interpreted as control instability. I traced it instead to packet misalignment on UART reads. Without strong framing, the host could start decoding from the middle of a packet, which corrupted the floats. I added an explicit two-byte sync header and updated the host to scan for it before unpacking the struct. Then I found a second bug: because the STM32 is little-endian, I had to correct the literal value so the on-wire byte order matched what the host expected. That removed both the garbage data and a host-side hang condition.

## Q4. How did you validate firmware without full hardware?

### Answer

I used a hardware-in-the-loop approach. The real firmware ran on the STM32 board, but instead of depending only on physical sensors or a full integrated setup, a Python host simulated the satellite dynamics and sent sensor-like inputs to the firmware over a binary protocol. The firmware sent back command voltages and telemetry. That gave me a realistic closed loop for validation, so I could debug communication, current control, actuator limits, and controller behavior before relying on full system integration.

## Q5. What did your control architecture look like?

### Answer

I treated it as an outer-loop and inner-loop problem. The outer loop computed the requested magnetic dipole based on the control mode, like B-dot detumbling or quaternion-based pointing. The inner loop translated that into target currents and used PI controllers to command actuator voltage on each axis. In HITL, I also exposed telemetry for the dipole request, projected torque, and saturation state so I could see not just what the controller wanted, but what was physically achievable.

## Q6. What is B-dot control and how did you implement it?

### Answer

B-dot detumbling is based on damping rotational motion using magnetic torquers. In my implementation I used gyro-estimated `cross(w, B)` rather than a noisy finite-difference approximation for `B_dot`. I then mapped that into a dipole command with a tuned gain. I also added a kick when the satellite spin was high but the estimated `B_dot` was very small, because that usually meant the angular velocity was aligned with the magnetic field and the controller could get stuck in a weak-control geometry.

## Q7. How did you handle actuator constraints in pointing mode?

### Answer

Magnetorquers can only generate torque perpendicular to the magnetic field, so not every desired torque is achievable at every instant. In the pointing controller, I projected the requested torque onto the plane orthogonal to the magnetic field and measured projection loss. If the geometry was poor, I limited integral growth and in severe cases blended toward B-dot damping instead of pretending the actuator could do the impossible. That kept the controller better behaved under realistic magnetic constraints.

## Q8. What is one example where telemetry changed your conclusion?

### Answer

The biggest example was the HITL latency issue. Before I instrumented it properly, the system just felt sluggish. With better telemetry and plotting, I could see the response was based on stale buffered packets rather than current state. That changed the problem from “maybe my controller is weak” to “my communication path is giving me the wrong timing semantics.” So telemetry prevented me from tuning the wrong thing.

## Q9. Why did you move from bare-metal to FreeRTOS?

### Answer

The original structure was fine for early bring-up, but it put too much responsibility into a single super-loop and feature-flag-heavy `main.c`. As the project grew to include sensor servicing, control logic, telemetry, and simulation support, FreeRTOS gave me a cleaner way to separate responsibilities and reason about timing. I moved the behavior into `app_runtime.c`, created dedicated tasks, and made sure the control path still had explicit safety handling, like zeroing outputs if host packets stopped arriving.

## Q10. How do you approach debugging in embedded systems?

### Answer

I try to identify the narrowest observable failure mode first, then instrument the boundaries around it. In this project that often meant asking whether a symptom came from the firmware logic, the communication path, the simulator, or the physical model. I rely heavily on telemetry, logs, and controlled flags to isolate one variable at a time. I try not to tune or rewrite anything until I can explain the failure in system terms.

## Q11. What C or embedded fundamentals would you highlight from this project?

### Answer

Interrupt-driven UART receive state machines, binary packet struct handling, endianness issues, DMA-backed logging, timer-driven control loops, SPI/I2C driver integration, and careful management of safety limits and shared runtime state. Later, the RTOS version also let me work with task boundaries and periodic scheduling semantics.

## Q12. If I asked you to debug a firmware issue in production, what would you do first?

### Answer

First I’d determine the exact symptom and whether it is deterministic, intermittent, timing-related, or data-related. Then I’d identify the narrowest part of the system where I can observe ground truth. In my CubeSat project that usually meant validating packet integrity, timestamps, or sensor readings before changing controller logic. I’d rather rule out observability, timing, and interface assumptions first than make speculative fixes deeper in the stack.

## 7. Behavioral interview questions with tailored answers

## Q1. Tell me about a time you faced a setback.

### Answer

During the CubeSat hardware-in-the-loop work, I had a period where the system looked unstable and untrustworthy. I was seeing lag, stale responses, and sometimes absurd values in the telemetry. It would have been easy to assume the controller itself was wrong, but I stepped back and treated it as a system-debugging problem. I isolated the communication path, timing, and numerical model separately and found several distinct issues: stale serial buffering, packet misalignment, and a simulation stability problem. Fixing them one by one turned the system from something that felt chaotic into something I could actually trust. That taught me not to collapse multiple symptoms into one assumed root cause.

## Q2. Tell me about a time you had to learn something quickly.

### Answer

The IMU bring-up phase is a good example. I had to get comfortable quickly with the BNO085’s SPI/SHTP protocol details, including packet continuation behavior, report parsing, and startup sequencing. I did not start as an expert in that specific device protocol, so I had to read the structure carefully, observe the device behavior, and test assumptions incrementally. That effort paid off because stable IMU communication became foundational for the rest of the project.

## Q3. Tell me about a time you improved a process or design.

### Answer

I improved both the validation process and the runtime architecture. On the validation side, I built the HITL setup so I could test firmware against a realistic plant instead of relying only on isolated firmware tests. On the architecture side, once the project outgrew the original super-loop, I moved the runtime into a FreeRTOS-based structure with clearer task separation. In both cases the goal was the same: make the system easier to reason about and easier to trust.

## Q4. Tell me about a time you handled ambiguity.

### Answer

The CubeSat project had ambiguity in both the physics and the firmware behavior. Sometimes it wasn’t obvious whether a problem was due to a controller choice, a plant assumption, a protocol bug, or missing instrumentation. My way of handling that was to reduce ambiguity by creating observable interfaces: telemetry, sync headers, explicit runtime flags, and a more formal separation between host and firmware responsibilities. Once the interfaces were cleaner, the ambiguity dropped and decisions became more defensible.

## Q5. Tell me about a conflict or disagreement on a team.

### Tailored answer

One realistic disagreement in the CubeSat project was around how much we should trust higher-level closed-loop ADCS behavior before the inner current-control path and actuator calibration were fully validated. In other words, part of the tension was open-loop versus closed-loop thinking. One approach was to move faster by focusing directly on top-level controller tuning. My concern was that if current tracking, voltage limits, or dipole conversion were off, then some “control problems” would actually be actuator-path or calibration problems in disguise.

What helped was reframing the disagreement away from “who is right” and toward “what experiment will separate these hypotheses.” I kept open-loop support in the system as a diagnostic mode, added telemetry for target current, measured current, voltage command, and later saturation and projection-related signals, and used HITL runs to compare behaviors under both assumptions. That let the team treat open-loop as a debugging tool and closed-loop as the real operating mode, instead of arguing for one as a philosophy. The result was a more disciplined validation sequence and more trust in the tuning data. That experience taught me that in technical conflict, I’m most effective when I turn the disagreement into a measurable interface question and propose the next test.

If they push for a more interpersonal version:

I’m generally pretty direct technically, so one thing I’ve learned is that being right is not enough if the team doesn’t share the same picture of the problem. I try to slow down, restate the other person’s concern accurately, and then use concrete evidence to move the conversation forward. In engineering teams, I think that balance matters: strong technical opinions, but low ego about how we get to the answer.

### Shorter version

We had a technical disagreement around how quickly to rely on closed-loop ADCS behavior before the inner current-control path was fully validated. I pushed for keeping open-loop available as a diagnostic mode so we could separate controller issues from actuator or calibration issues. I added telemetry and used HITL runs to compare the behaviors, which helped the team align on a better validation sequence. My general approach to conflict is to make the disagreement measurable.

## Q6. Tell me about a time you contributed to a team.

### Answer

I contribute best by creating clarity around technical systems. In the CubeSat project that meant documenting architecture, control flow, and transition guides in addition to writing code. I added docs around the HITL framework, the stage-1 physics model, the stage-2 firmware architecture, and later the RTOS transition. That kind of work helps the team move faster because it reduces the amount of tribal knowledge trapped in one person’s head.

## Q7. Tell me about a time you made a mistake.

### Answer

One clear example was in the serial sync work, where the header value on the firmware side produced the reverse byte order on the wire compared to what the host expected. The initial framing idea was right, but the exact implementation still had an endian mismatch. What mattered is that I caught it by checking the wire-level behavior rather than assuming the constant in code meant the bytes would appear in that order. That reinforced a lesson I use a lot now: for embedded interfaces, verify the actual on-wire representation, not just the source-level intent.

## Q8. Why Western Digital?

### Answer

What attracts me is that this role sits at the intersection of software and hardware, which is where I’ve done my strongest work. In my CubeSat project, the most interesting problems were never purely abstract software problems. They involved protocols, timing, sensor interfaces, actuator limits, and validation under realistic system behavior. Western Digital firmware work seems similar in that the software has to be correct, but it also has to respect the underlying hardware and system constraints. That’s the environment I want to grow in.

## 8. Teamwork and collaboration framing

When asked how you work with teams, stay close to this framing:

- I like making system boundaries explicit.
- I communicate through concrete artifacts: telemetry, docs, diagrams, interface definitions.
- I prefer small experiments over opinion battles.
- I’m comfortable owning low-level debugging, but I also make sure the result becomes team-readable.

### Short scripted answer

I’m at my best on technical teams when I can reduce ambiguity. I do that by making interfaces and evidence visible, whether that’s logs, telemetry, protocol definitions, or docs. In the CubeSat project, I found that a lot of progress came not just from writing the fix, but from making the system easier for everyone else to understand and validate. That’s how I try to work with teams: rigorous on the technical details, but collaborative in how I surface them.

### Realistic UCI CubeSat team context

Keep the team framing realistic and accurate:

- The public UCI CubeSat team page shows a larger organization with Executive, Advisory, Avionics, Communications, Power, Structures, and Systems groups.
- Within that org, you are listed under Avionics Software.
- Your own system notes show real technical interfaces with power, structure, and ADCS/control concerns.

That means a stronger way to describe the team is:

- “UCI CubeSat is a larger student engineering organization, and I worked on the Avionics software side. Even within Avionics, a lot of our decisions had to line up with power and actuator constraints, structure and inertia assumptions, and system-level integration.”

That phrasing is better than calling it a small team, and it keeps your role specific and credible.

### If they ask specifically about conflict within a subteam

Use this version:

Within the broader UCI CubeSat team, I was on the Avionics software side, but our work still depended on assumptions from other subteams. Some questions were really control questions, some were power and actuator-authority questions, and some were structure and inertia-assumption questions. One conflict we had was around how to interpret poor closed-loop behavior. It would have been easy to say the gains were wrong, but I was concerned that if voltage limits, coil constants, or current tracking were off, then we would tune around the wrong problem. I handled that by making the disagreement concrete: keep open-loop available as a diagnostic path, add telemetry for current, voltage, and saturation, and compare controlled HITL runs. That shifted the conversation from opinions to evidence and helped us agree on a better validation sequence.

## 9. Questions they may ask about code quality, tradeoffs, or decisions

## Why not just test everything on hardware?

Because full hardware-only iteration is slower, harder to instrument, and less controllable. HITL gave me a way to preserve real firmware behavior while varying scenarios and observing controller internals more directly.

## Why parse firmware constants from `config.h` on the host?

To reduce silent drift between host assumptions and firmware assumptions. If voltage or dipole conversion is inconsistent between the two, tuning results become misleading.

## Why branch-separate simulation and sensor demos instead of one compile flag?

Because once the demo paths diverged enough, branch-level separation became cleaner and less error-prone than maintaining a growing matrix of conditional paths inside the same runtime.

## Why keep open-loop mode around?

For controlled debugging and consistency checks. Open-loop lets me isolate actuator-path or model-path issues without PI dynamics obscuring the symptom.

## 10. Quick-hit fundamentals to review before the interview

- pointers, structs, const, volatile
- static storage duration and scope in C
- interrupt safety and shared state
- endianness
- UART/SPI/I2C tradeoffs
- timers, PWM, DMA
- task scheduling basics in FreeRTOS
- PI/PID intuition
- sensor bring-up workflow
- binary protocol versioning

## 11. Short answers for common rapid-fire questions

## What language are you most comfortable using for firmware?

Embedded C. That’s what I used for the STM32 side of the CubeSat project.

## Have you worked with RTOS?

Yes. I migrated the project runtime from a super-loop model into a FreeRTOS task-based structure and separated control, sensor, IMU, and logging responsibilities.

## Have you debugged hardware/software interface issues?

Yes. The strongest examples are SPI-based IMU bring-up and UART binary protocol debugging in the HITL system.

## Have you worked on performance or timing issues?

Yes. I debugged stale-data latency caused by loop-rate mismatch and serial buffering, and I also handled timing-related control/runtime concerns when moving to RTOS.

## Have you designed protocols?

Yes. I designed and evolved a binary host/firmware packet format with sync bytes, quantized telemetry fields, runtime flags, and version checks.

## 12. Questions to ask the interviewer

- How much of the role is bring-up/debugging versus adding new firmware features?
- What are the main hardware interfaces or subsystems this firmware team owns?
- How does the team validate firmware changes: hardware benches, simulation, CI, or all of the above?
- For early-career engineers, what separates someone who ramps up well from someone who struggles?
- How much interaction does the firmware team have with validation, hardware, and systems teams?

## 13. Interview traps to avoid

- Do not oversell aerospace or controls theory if they ask a firmware question. Lead with embedded execution details.
- Do not present the project as purely solo genius work. Emphasize system clarity, docs, and evidence-driven collaboration.
- Do not say “I just changed parameters until it worked.” Always describe a hypothesis and validation path.
- Do not get lost in quaternion math unless they explicitly lean there.
- Do not open with “I’m mostly software.” Open with embedded systems and firmware evidence.

## 14. What to emphasize most

If time is short, bias toward these three:

- I debugged real embedded communication and timing issues.
- I built a HITL setup to validate firmware realistically.
- I refactored the runtime into a more disciplined RTOS architecture.

Those are the most credible bridges from your CubeSat experience to a Western Digital firmware role.
