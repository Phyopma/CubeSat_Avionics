# CubeSat Nucleo Firmware

Firmware for the CubeSat Magnetorquer Control System, running on the **STM32L476RG Nucleo** board. This project implements a high-frequency (1kHz) current control loop for magnetorquers, along with sensor integration for attitude determination.

## Architecture

The system is built on the STM32 HAL, with a layered architecture separating hardware drivers from control algorithms.

<details>
<summary>Click to view Architecture Diagram</summary>

```mermaid
graph TB
    subgraph Hardware
        MCU[STM32L476RG MCU]
        IMU[BNO085 IMU]
        TEMP[ADT7420 Temp Sensor]
        MOTOR[H-Bridge / Actuator]
        SENS[Current Sensor]
        PC[PC / Telemetry]
    end

    subgraph Firmware_Core
        HAL[STM32 HAL]
        DMA[DMA Controller]
        UART[UART Driver]
        I2C[I2C Driver]
        SPI[SPI Driver]
        TIM[Timer Driver]
    end

    subgraph Application_Drivers
        BNO_DRV[BNO085 Driver]
        ADT_DRV[ADT7420 Driver]
        HB_DRV[H-Bridge Driver]
        CS_DRV[Current Sensor Driver]
    end

    subgraph Algorithms
        CTRL[Inner Loop Control]
        PI[PI Controller]
    end

    subgraph Utilities
        LOG[Serial Logger]
        PLOT[Teleplot]
    end

    %% Hardware Connections
    MCU <-->|I2C| TEMP
    MCU <-->|SPI| IMU
    MCU <-->|PWM| MOTOR
    MCU <-->|ADC| SENS
    MCU <-->|UART| PC

    %% Firmware Layers
    HAL <--> UART
    HAL <--> I2C
    HAL <--> SPI
    HAL <--> TIM

    %% App Drivers -> HAL
    BNO_DRV --> SPI
    ADT_DRV --> I2C
    HB_DRV --> TIM
    CS_DRV --> HAL

    %% Algorithms -> Drivers
    CTRL --> BNO_DRV
    CTRL --> CS_DRV
    CTRL --> HB_DRV
    CTRL --> PI

    %% Utilities
    LOG --> UART
    PLOT --> LOG

    %% Main Execution
    CTRL -.->|1kHz IRQ| TIM
```
</details>

### Data Flow

The **Inner Control Loop** runs at 1kHz (triggered by TIM6), reading sensors, executing the PI controller, and updating the H-Bridge PWM commands.

<details>
<summary>Click to view Data Flow Diagram</summary>

```mermaid
sequenceDiagram
    participant TIM as Timer (1kHz)
    participant CTRL as InnerControlLoop
    participant SENS as Sensors (IMU/Current)
    participant ALG as PI Controller
    participant ACT as H-Bridge
    participant TELE as Teleplot/UART

    Note over TIM, TELE: 1kHz Control Cycle

    TIM->>CTRL: PeriodElapsedCallback (IRQ)
    activate CTRL
    
    CTRL->>SENS: Read Feedback (Current/IMU)
    SENS-->>CTRL: Raw Data
    
    CTRL->>ALG: Update PI(Target, Measured)
    activate ALG
    ALG-->>CTRL: Voltage Command
    deactivate ALG
    
    CTRL->>ACT: Set PWM Duty Cycle
    
    CTRL->>TELE: Queue Telemetry (Target, Current, Voltage)
    deactivate CTRL

    Note right of TELE: DMA Transfer (Async)
```
</details>

## Features

- **Inner Loop Control**: Quick-response PI controller for magnetorquer current regulation (`Application/Algorithms/`).
- **IMU Integration**: BNO085 driver support for Rotation Vector, Gyroscope, and Magnetometer data.
- **Temperature Sensing**: ADT7420 High-Accuracy Digital Temperature Sensor.
- **Telemetry**: Real-time plotting support via [Teleplot](https://teleplot.fr/) over UART DMA.

## Documentation

- [Telemetry & Control Interfaces](./cube_sat_nucleo/api-endpoints.md)
- [Project Architecture](./cube_sat_nucleo/architecture.mmd)

## Project Structure

```text
├── Core/                   # STM32 HAL System Initialization & Interrupts
│   ├── Src/main.c         # Application Entry & Loop
│   └── Src/serial_log...  # DMA-driven UART Logger
├── Application/
│   ├── Drivers/           # Hardware Drivers
│   │   ├── imu_bno085.c   # BNO085 IMU Driver
│   │   ├── adt7420.c      # ADT7420 Temp Sensor
│   │   └── hbridge.c      # PWM H-Bridge Control
│   └── Algorithms/        # Control Logic
│       ├── pi_controller.c # Proportional-Integral Controller
│       └── inner_loop...  # Main Control Loop State Machine
└── Drivers/               # STM32 CMSIS & HAL Drivers
```

## Recent Changes

- **Control Loop**: Implemented and tuned Inner Loop PI controller; added Open Loop testing flags.
- **Sensors**: Added retry logic for IMU activation; Thermocouple testing completed.
- **Comms**: Fixed Serial Output buffer issues; Implemented DMA-based logging.
- **Config**: Cleaned up `.gitignore` and build artifacts.
