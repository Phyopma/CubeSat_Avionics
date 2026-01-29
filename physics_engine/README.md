# Physics Engine (HITL Host)

This directory contains the Python-based "Physics Engine" for the Hardware-in-the-Loop (HITL) simulation. It simulates the CubeSat's environment and dynamics, communicating with the STM32 Nucleo firmware over a binary UART protocol.

## Prerequisites

*   [uv](https://github.com/astral-sh/uv) (for dependency management)
*   Python 3.12+

## Setup

1.  **Install Dependencies**:
    ```bash
    uv sync
    ```

2.  **Configure Serial Port**:
    Edit `simulation_host.py` and set `SERIAL_PORT` to your Nucleo's device path (e.g., `/dev/tty.usbmodem...` or `COM3`).

## Usage

Run the simulation using `uv run`. You can select different waveform modes to verify the Nucleo's control loop.

### 1. Step Response (Square Wave)
Great for tuning PI gains.
```bash
uv run simulation_host.py --mode step --amp 0.05 --freq 0.5
```

### 2. Sine Wave
```bash
uv run simulation_host.py --mode sine --amp 0.05 --freq 1.0
```

### 3. Random Targets
```bash
uv run simulation_host.py --mode random
```

## Protocol
The host communicates using a packed binary struct (Little-endian):
*   **Input (Host -> Firmware)**: `48 bytes`
    *   `float current_amps` (Simulated Feedback)
    *   `float gyro[3]` (Simulated Gyro)
    *   `float mag[3]` (Simulated Mag)
    *   `float quat[4]` (Simulated Orientation)
    *   `float target_current_cmd` (Requested Setpoint)
*   **Output (Firmware -> Host)**: `8 bytes`
    *   `float command_voltage`
    *   `float debug_val`
