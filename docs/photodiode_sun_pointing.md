# Photodiode Sun-Pointing Sensor Plan

## Scope

This note documents the firmware scaffold for four photodiodes used as a
coarse Sun sensor. The current implementation keeps the hardware access layer
as a placeholder because `cube_sat_nucleo.ioc` does not yet generate ADC
peripherals. The estimation and ADCS target-vector plumbing are implemented so
pin bring-up can be limited to the driver.

## Current Firmware Shape

- Driver placeholder:
  - `cube_sat_nucleo/CM7/Application/Drivers/Inc/photodiode.h`
  - `cube_sat_nucleo/CM7/Application/Drivers/Src/photodiode.c`
- Runtime estimator:
  - `cube_sat_nucleo/CM7/Application/Algorithms/Inc/photodiode_runtime.h`
  - `cube_sat_nucleo/CM7/Application/Algorithms/Src/photodiode_runtime.c`
- ADCS target selection:
  - `adcs_sensor_input_t.pointing_target`
  - `POINTING_TARGET_INERTIAL`
  - `POINTING_TARGET_SUN`
  - `POINTING_TARGET_EARTH`

The default target source remains the legacy inertial/quaternion path:

```c
#define ADCS_DEFAULT_POINTING_TARGET_SOURCE 0U
```

Set it to `1U` after ADC bring-up to command Sun pointing from the photodiode
vector.

## Suggested ADC Pin Candidates

The active `.ioc` already uses:

| Function | Pins |
| --- | --- |
| TIM2 PWM | PA0, PA1 |
| USART2 | PA2, PA3 |
| MTQ GPIO | PA4, PA5 |
| IMU GPIO | PB2, PB5, PA10 |
| I2C1 | PB8, PB9 |
| I2C2 | PB10, PB11 |
| SPI3 | PC10, PC11, PC12 |

Suggested non-conflicting analog candidates for NUCLEO-H755ZI-Q are:

| Diode | Suggested MCU pin | ADC input | Notes |
| --- | --- | --- | --- |
| D0 | PA6 | ADC12_INP3 | Verify Nucleo header and analog routing before wiring. |
| D1 | PC0 | ADC123_INP10 | Candidate shared by ADC1/2/3. |
| D2 | PC1 | ADC123_INP11 | Candidate shared by ADC1/2/3. |
| D3 | PC2 | ADC123_INP12 | Candidate shared by ADC1/2/3; check PC2 vs PC2_C in CubeMX. |

These are comments in `photodiode.c`, not a final board decision. Regenerate
CubeMX after selecting the actual pins and add the generated `adc.c/.h` files
to the CM7 build.

## ADC / Clock Notes

The current RCC records `RCC.ADCFreq_Value=129000000`, but no ADC instance is
enabled. When ADC is enabled:

- Configure four regular channels, preferably on one ADC instance in scan mode.
- Use a stable sample time long enough for the photodiode/TIA source impedance.
- Calibrate ADC single-ended mode at boot.
- If polling, keep the read path below the sensor task budget.
- If DMA, publish one coherent 4-channel sample after a full conversion group.

The STM32CubeH7 HAL sequence for polling is:

```c
HAL_ADC_Start(&hadcX);
HAL_ADC_PollForConversion(&hadcX, timeout);
raw = HAL_ADC_GetValue(&hadcX);
HAL_ADC_Stop(&hadcX);
```

An external I2C/SPI ADC is also acceptable. Keep the same `Photodiode_ReadRaw`
interface and move bus-specific reads behind it.

## Normal Vectors

Each photodiode has a configurable body-frame normal:

```c
photodiode_calibration_t.normal_body
```

The default scaffold assumes four non-parallel normals tilted around a +Z
panel boresight:

```text
D0: normalize(+0.30, +0.30, +1.00)
D1: normalize(-0.30, +0.30, +1.00)
D2: normalize(-0.30, -0.30, +1.00)
D3: normalize(+0.30, -0.30, +1.00)
```

This is intentionally easy to replace. If the final geometry places all four
photodiodes parallel to the panel normal, the system cannot infer a Sun vector;
it can only infer brightness along that one axis. Vector estimation requires
non-parallel known normals or a calibrated quadrant-sensor optical layout.

## Sun Vector Estimator

For each channel:

```text
signal_i = raw_i - dark_offset_i
m_i = clamp(signal_i / gain_i, 0, 1)
m_i ~= max(0, normal_i dot sun_body)
```

With at least three valid non-degenerate channels, the runtime solves:

```text
sun_raw = inverse(N^T W N) N^T W m
sun_body = normalize(sun_raw)
```

Fallback behavior:

| Valid channels | Behavior |
| --- | --- |
| 4 | Full least-squares Sun vector. |
| 3 | Full least-squares Sun vector with one failed diode. |
| 2 | Degraded weighted-normal backup vector. |
| 1 | Hold recent vector if fresh, otherwise use the single diode normal as a low-quality backup. |
| 0 | Hold recent vector briefly, otherwise mark Sun vector invalid. |

This means one or two diode failures do not take down the whole ADCS path. The
snapshot carries `sun_valid`, `degraded`, `quality`, `valid_mask`,
`saturated_mask`, and `failed_mask` so control and telemetry can decide how
aggressive to be.

## ADCS Control Integration

The existing pointing controller was refactored around a body-frame target
vector:

```text
err = body_axis x target_body
tau_raw = Kp*err + Ki*integral - Kd*gyro
tau_proj = tau_raw projected perpendicular to magnetic field
m_cmd = (B x tau_proj) / |B|^2
```

Sun pointing sets:

```text
body_axis = configured spacecraft face, default +Z
target_body = photodiode_snapshot.sun_body
```

If the Sun vector is invalid, pointing falls back to B-dot damping instead of
using stale geometry silently.

## Earth Pointing

Photodiodes do not provide Earth direction. They only measure Sun direction in
the body frame. The IMU gyroscope reports rotation rate, and the BNO085
rotation vector can be useful for lab orientation tests, but in orbit an
accelerometer is not a reliable gravity/down vector because the spacecraft is
in free fall.

Earth pointing needs one of these target sources:

- orbit/TLE/GPS + time to compute nadir in an inertial/orbital frame, then an
  attitude estimate to rotate nadir into body frame
- an Earth horizon sensor, IR sensor, or camera
- a fused estimator such as TRIAD/QUEST/EKF using Sun vector, magnetometer,
  and orbit model

The firmware has `POINTING_TARGET_EARTH` as a placeholder, but it deliberately
marks the vector invalid until a real nadir source is added.

## References

- Joseph C. Springmann and James W. Cutler, photodiode coarse Sun sensor model.
- University of Michigan photodiode placement material.
- A Guide to CubeSat Mission and Bus Design, Section 8.6 sensors.
- BNO08X datasheet: BNO085 integrates accelerometer, gyroscope, magnetometer,
  and sensor-fusion rotation-vector reports over SPI/I2C/UART-SHTP.
