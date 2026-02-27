# CubeSat RTOS Transition Guide: Bare Metal to FreeRTOS

This guide explains the architectural shift from a bare-metal "Super-Loop" to a multitasking FreeRTOS environment in the CubeSat project. If you've been working with the previous bare-metal commit (`d6722d4`), this document will help you master the new code in `ca16460`.

---

## 1. The Core Shift: Super-Loop vs. Multitasking

### Bare Metal (Super-Loop)
In the previous version, everything happened in one big `while(1)` loop in `main.c`.
```c
// Bare Metal Mental Model: "Sequential"
while (1) {
    ReadSensors();    // If this takes 10ms, control lags!
    RunControl();     // Runs at whatever frequency the loop allows.
    SendTelemetry();  // If UART is slow, loop stalls.
}
```
**Problem:** A slow sensor read or a long UART transmit directly delays your high-priority control loop.

### RTOS (Concurrent Tasks)
In the current version, the work is split into independent **Tasks** (threads). Each task thinks it has the CPU to itself.
```c
// RTOS Mental Model: "Parallel"
void ControlTask() { 
    for(;;) { run_control(); vTaskDelay(1); } // Runs every 1ms
}
void TelemetryTask() {
    for(;;) { send_data(); vTaskDelay(50); } // Runs every 50ms
}
```
**Solution:** The RTOS **Scheduler** pauses low-priority tasks (Telemetry) instantly if a high-priority task (Control) needs to run. This is called **Preemption**.

---

## 2. The New Codebase Structure (`AppRuntime`)

We transitioned from `main.c` bloat to a modular `AppRuntime` system found in `Core/Src/app_runtime.c`.

### Task Breakdown & Priorities
We now have 5 distinct tasks running at different levels:

| Task Name | Period | Priority | Purpose |
| :--- | :--- | :--- | :--- |
| **ControlTask** | 1ms | 5 (Highest) | Magnetorquer PI loop. Must be deterministic. |
| **CurrentTask** | 1ms | 4 | Handles asynchronous I2C sampling for the sensor. |
| **ImuTask** | 2ms | 4 | Services the BNO085 IMU data. |
| **TelemetryTask** | 50ms | 3 | Logs data to Teleplot/Python. |
| **LoggerTask** | ~1ms | 2 (Lowest) | Drains the DMA log buffer. |

---

## 3. Mastering the "Mental Model" Changes

### A. Preemption & Determinism
In bare metal, "determinism" was achieved by keeping the loop fast. In RTOS, it is guaranteed by **Priority**.
- **Rule:** If `ControlTask` (Pri 5) wakes up while `TelemetryTask` (Pri 3) is halfway through a `printf`, the CPU *instantly* context-switches to the Control Task.
- **Keep in mind:** High-priority tasks should be short. If they hog the CPU, low-priority tasks will "starve" and never run.

### B. Asynchronous Operations
Look at `current_sensor.c`. We no longer wait for I2C.
1. `ControlTask` calls `SubmitSampleRequest()`.
2. `CurrentTask` (running independently) sees the request, reads the sensor, and caches the result.
3. Next time `ControlTask` runs, it calls `GetLatestSample()` to get the *most recent cached data*.

### C. Resource Safety (Critical Sections)
Since tasks can stop each other at any line of code, sharing variables like `g_current_amps` is dangerous.
- **RTOS Concept:** Use **Atomics** or **Disable Interrupts** when reading/writing shared state.
- **Example in code:** `InnerLoop_GetStateSnapshot` copies the state into a local buffer so the caller has a consistent "picture" of the system that won't change mid-read.

---

## 4. Master the Changes: Step-by-Step

1.  **Study `app_runtime.c`**: See how `vTaskStartScheduler()` hands over control from `main()` to the RTOS.
2.  **Compare `HAL_Delay` vs `vTaskDelay`**: 
    - `HAL_Delay(10)` spins the CPU, wasting power and blocking other code.
    - `vTaskDelay(10)` puts the task to sleep, letting other tasks use the CPU.
3.  **Trace a Control Tick**:
    - `TIM6_IRQHandler` (Interrupt) -> `AppRuntime_OnControlTickFromISR` -> `vTaskNotifyGiveFromISR`.
    - This "kicks" the `ControlTask` out of its sleep immediately.

---

## 5. Pro-Tips for Debugging RTOS

1.  **Stack Overflow**: RTOS tasks have small, fixed stacks (~512 - 1024 bytes). If you declare a `char buffer[2048]`, you will crash. Use `static` or `heap` for large arrays.
2.  **Priority Inversion**: Never let a high-priority task wait on a resource (like a Mutex) held by a low-priority task.
3.  **ISR Safety**: Never call a "blocking" RTOS function inside an interrupt. Always use the `FromISR` version (e.g., `xQueueSendToBackFromISR`).

---

## 6. Deep Dive: Blocking vs. ISRs

To master RTOS, you must understand exactly what "Blocking" means and why Interrupts (ISRs) are a "special zone."

### What is a "Blocking" Function?
In bare metal, "blocking" usually meant a busy-wait (e.g., `while(flag == 0);`). This wastes CPU cycles.
In an RTOS, **Blocking** means the task voluntarily gives up its CPU time.
- **How it works:** When you call `vTaskDelay(10)` or `xQueueReceive(queue, &data, portMAX_DELAY)`, the task tells the Scheduler: *"I am now in the **BLOCKED** state. Please take me off the 'Ready' list and give the CPU to someone else until the time passes or the queue has data."*
- **Result:** Even though your code looks like an infinite `while(1)`, it is actually "sleeping" most of the time, letting lower-priority tasks run.

### Why ISRs are Different (The "FromISR" Rule)
You might think: *"If an ISR blocks, shouldn't the scheduler just switch to another task?"*
**No.** Here is why:

1.  **ISRs are not Tasks:** Tasks are managed by the RTOS software. ISRs are triggered by the hardware. An ISR is "above" the RTOS.
2.  **ISRs Preempt the Scheduler:** When an hardware interrupt happens, the CPU stops *everything*—including the Scheduler—to run the ISR. If the ISR tries to "block" (sleep), the CPU would just hang because the thing that handles "sleeping" (the scheduler) is currently paused by the ISR!
3.  **No Context Saving:** Tasks have a "Stack" where their state is saved during a switch. ISRs often share a single system stack. They don't have a way to "pause and come back later" like a task does.

### RTOS Functions: Task vs. ISR
FreeRTOS provides two versions of many functions:
- **Standard (`xQueueSend`):** Can be called from a Task. It might "block" if the queue is full.
- **ISR Version (`xQueueSendFromISR`):** 
    - **Never Blocks:** If the queue is full, it returns an error immediately. It *must* return so the ISR can finish.
    - **Context Switch Flag:** It takes a pointer (`pxHigherPriorityTaskWoken`). If sending the message wakes up a high-priority task, the ISR will trigger a context switch *exactly as it finishes*, ensuring the system stays responsive.

**Golden Rule:** If you are inside a function starting with `HAL_..._Callback` or any interrupt handler, **ONLY** use `FromISR` functions.

---

*This document is part of the CubeSat Avionics knowledge base.*
