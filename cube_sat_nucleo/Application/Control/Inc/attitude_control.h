/*
 * attitude_control.h
 *
 *  Created on: Nov 3, 2025
 *      Author: phyopyae
 */

#ifndef APPLICATION_CONTROL_INC_ATTITUDE_CONTROL_H_
#define APPLICATION_CONTROL_INC_ATTITUDE_CONTROL_H_

#include "stm32l4xx_hal.h"
#include "adt7420.h"
#include "imu_bno085.h"
#include "magnetorquer.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// CubeMX timer instance for 10 Hz control loop
extern TIM_HandleTypeDef htim1; // Configure in CubeMX for 100ms interrupts

// Control algorithm constants (from Cornell Alpha implementation)
#define ACS_CONTROL_FREQUENCY_HZ 10.0f  // Control loop frequency
#define ACS_CONTROL_PERIOD_MS 100       // Control loop period
#define ACS_TARGET_SPIN_RATE_RAD_S 0.5f // Target Z-axis spin rate (rad/s)
#define ACS_DAMPING_COEFFICIENT 1.0f    // Kane damper coefficient
#define ACS_DAMPER_INERTIA 0.01f        // Kane damper inertia (kg⋅m²)
#define ACS_MAX_DETUMBLE_RATE 0.3f      // Max X,Y angular rate for detumble success

// Earth magnetic field model (approximate)
#define EARTH_MAG_FIELD_STRENGTH_UT 25.0f // Typical Earth field strength (µT)
#define IGRF_MODEL_INCLINATION_DEG 60.0f  // Magnetic inclination (location dependent)

// Control modes (following Cornell implementation)
typedef enum
{
    ACS_MODE_IDLE = 0,       // System idle - no control
    ACS_MODE_DETUMBLE,       // Detumbling and spin-up phase
    ACS_MODE_SPIN_STABILIZE, // Spin stabilization mode
    ACS_MODE_POINT,          // Pointing mode (future enhancement)
    ACS_MODE_SAFE            // Safe mode - magnetorquers off
} acs_mode_t;

// Control status flags
typedef enum
{
    ACS_STATUS_OK = 0x00,
    ACS_STATUS_SENSOR_FAULT = 0x01,
    ACS_STATUS_ACTUATOR_FAULT = 0x02,
    ACS_STATUS_THERMAL_FAULT = 0x04,
    ACS_STATUS_POWER_FAULT = 0x08,
    ACS_STATUS_TIMEOUT = 0x10
} acs_status_t;

/**
 * @brief Sensor data structure (aligned with your existing drivers)
 */
typedef struct
{
    // Temperature sensor data (using your ADT7420_Data structure)
    ADT7420_Data temperature;

    // IMU sensor data (using your BNO085_Data structure)
    BNO085_Data imu;

    // Derived attitude data for control
    float angular_velocity[3]; // [ωx, ωy, ωz] body frame (rad/s)
    float earth_mag_field[3];  // Corrected Earth magnetic field (µT)
    float attitude_euler[3];   // [roll, pitch, yaw] (rad)

    // Data quality and timing
    uint8_t sensors_active; // Bitmask: bit0=temp, bit1=IMU
    uint8_t data_quality;   // Data quality score (0-100%)
    uint32_t timestamp_ms;  // System timestamp
    bool valid;             // Overall data validity
} acs_sensor_data_t;

/**
 * @brief Control algorithm output structure
 */
typedef struct
{
    // Magnetorquer commands (using your magnetorquer structure)
    magnetorquer_cmd_t mag_command;

    // Control algorithm state
    float control_torque[3]; // Computed control torque (N⋅m)
    float dipole_moment[3];  // Desired magnetic dipole (A⋅m²)
    acs_mode_t active_mode;  // Current control mode

    // Control performance metrics
    float spin_rate_error;   // Z-axis spin rate error (rad/s)
    float detumble_progress; // X,Y detumbling progress (0-1)
    float control_effort;    // Total control effort magnitude

    // System status
    acs_status_t status_flags;    // Status and fault flags
    float power_estimate_w;       // Estimated power consumption (W)
    uint32_t control_cycle_count; // Control iteration counter
    uint32_t timestamp_ms;        // Command timestamp
    bool valid;                   // Command validity
} acs_control_output_t;

/**
 * @brief Kane damper control parameters (from Cornell characterization)
 */
typedef struct
{
    float damper_inertia;        // Virtual damper inertia (kg⋅m²)
    float damping_coefficient;   // Damping coefficient
    float target_spin_rate;      // Target Z-axis spin rate (rad/s)
    float max_control_torque[3]; // Maximum torque per axis (N⋅m)
    bool parameters_valid;       // Parameter validation flag
} kane_damper_params_t;

/**
 * @brief Main attitude control system handle (following your driver pattern)
 */
typedef struct
{
    // Peripheral driver handles (your existing structures)
    ADT7420_Handle *temp_sensor;  // Temperature sensor handle
    bno085_t *imu_sensor;         // IMU sensor handle
    magnetorquer_t *magnetorquer; // Magnetorquer actuator handle

    // Control algorithm parameters
    kane_damper_params_t kane_params; // Kane damper parameters
    acs_mode_t current_mode;          // Current operational mode
    acs_mode_t requested_mode;        // Mode requested by flight software

    // Sensor data and control output
    acs_sensor_data_t sensor_data;       // Current sensor readings
    acs_control_output_t control_output; // Current control commands

    // Calibration and filtering (Cornell approach)
    float hard_iron_offset[3];        // Magnetometer hard-iron correction (µT)
    float mag_disturbance_poly[3][4]; // PWM disturbance polynomial coefficients
    float voltage_scale_factor;       // Battery voltage scaling factor
    float temp_compensation[3];       // Temperature drift compensation

    // System state and timing
    uint32_t last_control_update_ms;  // Last control loop execution time
    uint32_t mode_transition_time_ms; // Time of last mode change
    uint32_t total_runtime_ms;        // Total system runtime
    bool initialized;                 // Initialization status
    bool calibrated;                  // Calibration completion status
} acs_t;

// Public API functions (following your driver naming convention)
/**
 * @brief Initialize attitude control system
 * @param acs Pointer to ACS handle structure
 * @param temp Temperature sensor handle
 * @param imu IMU sensor handle
 * @param mag Magnetorquer handle
 * @return true on success, false on failure
 */
bool ACS_Init(acs_t *acs, ADT7420_Handle *temp, bno085_t *imu, magnetorquer_t *mag);

/**
 * @brief Main service function - call at 10 Hz from timer interrupt
 * @param acs Pointer to ACS handle structure
 * @return true on successful control cycle, false on error
 */
bool ACS_Service(acs_t *acs);

/**
 * @brief Set control system operational mode
 * @param acs Pointer to ACS handle structure
 * @param mode Desired operational mode
 * @return true on success, false on invalid mode
 */
bool ACS_SetMode(acs_t *acs, acs_mode_t mode);

/**
 * @brief Get current sensor data (thread-safe copy)
 * @param acs Pointer to ACS handle structure
 * @param data Pointer to store sensor data copy
 * @return true on success, false if data invalid
 */
bool ACS_GetSensorData(acs_t *acs, acs_sensor_data_t *data);

/**
 * @brief Get current control output (thread-safe copy)
 * @param acs Pointer to ACS handle structure
 * @param output Pointer to store control output copy
 * @return true on success, false if output invalid
 */
bool ACS_GetControlOutput(acs_t *acs, acs_control_output_t *output);

/**
 * @brief Perform hard-iron magnetometer calibration (Cornell method)
 * @param acs Pointer to ACS handle structure
 * @return true on successful calibration, false on failure
 */
bool ACS_CalibrateHardIron(acs_t *acs);

/**
 * @brief Load calibration parameters from flash memory
 * @param acs Pointer to ACS handle structure
 * @return true on success, false if calibration data invalid
 */
bool ACS_LoadCalibration(acs_t *acs);

/**
 * @brief Emergency stop - disable all actuators immediately
 * @param acs Pointer to ACS handle structure
 */
void ACS_EmergencyStop(acs_t *acs);

// Utility functions for control algorithm support
/**
 * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
 * @param quat Pointer to quaternion structure
 * @param euler Pointer to 3-element array for Euler angles (rad)
 */
void ACS_QuaternionToEuler(const bno085_quat_t *quat, float euler[3]);

/**
 * @brief Estimate angular velocity from quaternion time derivative
 * @param acs Pointer to ACS handle structure
 * @param current_quat Current quaternion reading
 */
void ACS_EstimateAngularVelocity(acs_t *acs, const bno085_quat_t *current_quat);

/**
 * @brief Apply magnetometer disturbance corrections (Cornell method)
 * @param acs Pointer to ACS handle structure
 * @param raw_field Raw magnetometer reading (µT)
 * @param corrected_field Corrected magnetic field output (µT)
 * @param battery_voltage Current battery voltage (V)
 */
void ACS_CorrectMagDisturbances(acs_t *acs, const float raw_field[3],
                                float corrected_field[3], float battery_voltage);

/**
 * @brief Kane damper control law implementation (Cornell algorithm)
 * @param params Kane damper parameters
 * @param omega_body Current body angular velocity (rad/s)
 * @param earth_field Earth magnetic field vector (µT)
 * @param control_torque Output control torque (N⋅m)
 */
void ACS_KaneDamperControl(const kane_damper_params_t *params,
                           const float omega_body[3],
                           const float earth_field[3],
                           float control_torque[3]);

/**
 * @brief Convert control torque to magnetic dipole moment
 * @param control_torque Control torque vector (N⋅m)
 * @param earth_field Earth magnetic field vector (µT)
 * @param dipole_moment Output magnetic dipole moment (A⋅m²)
 */
void ACS_TorqueToDipole(const float control_torque[3],
                        const float earth_field[3],
                        float dipole_moment[3]);

// Diagnostic and logging functions
/**
 * @brief Get system health status
 * @param acs Pointer to ACS handle structure
 * @return Health percentage (0-100%)
 */
uint8_t ACS_GetHealthStatus(acs_t *acs);

/**
 * @brief Log control system telemetry over UART
 * @param acs Pointer to ACS handle structure
 */
void ACS_LogTelemetry(acs_t *acs);

#endif /* APPLICATION_CONTROL_INC_ATTITUDE_CONTROL_H_ */
