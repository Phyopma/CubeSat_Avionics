# System Interfaces & Telemetry

## Telemetry (Teleplot)

The firmware streams telemetry data over UART (DMA-driven) formatted for [Teleplot](https://teleplot.fr/).

| Key | Unit | Description |
|-----|------|-------------|
| `Target` | mA | Target current setpoint for the PI controller. |
| `Current` | mA | Measured current from the INA219/Sensor. |
| `Error` | mA | Difference between Target and Measured current. |
| `Voltage` | V | Control voltage applied to the H-Bridge. |
| `Temp` | C | Board temperature (ADT7420). |
| `[RV]` | - | Rotation Vector Quaternion (Real, I, J, K). |
| `[GM]` | - | Game Rotation Vector (No Magnetometer). |
| `[GY]` | rad/s | Gyroscope data (X, Y, Z). |
| `[MG]` | uT | Magnetometer data (X, Y, Z). |
| `[LA]` | m/s^2 | Linear Acceleration. |

## Internal Control API

Key functions for controlling the magnetorquer loop (`inner_loop_control.h`):

### `InnerLoop_Init(void)`
Initializes the inner control loop drivers and logic.

### `InnerLoop_SetTargetCurrent(float current_amps)`
Sets the target current for the magnetorquer.
- **Parameters**: `current_amps` - Target current in Amperes.

### `InnerLoop_GetState(void)`
Returns the current state struct for debugging/plotting:
```c
typedef struct {
    float target_current;
    float measured_current;
    float command_voltage;
} mtq_state_t;
```
