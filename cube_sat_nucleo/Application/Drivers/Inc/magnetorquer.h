/*
 * magnetorquer.h
 *
 *  Created on: Nov 3, 2025
 *      Author: phyopyae
 */

#ifndef APPLICATION_DRIVERS_INC_MAGNETORQUER_H_
#define APPLICATION_DRIVERS_INC_MAGNETORQUER_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// CubeMX timer instance (configure in CubeMX for PWM generation)
extern TIM_HandleTypeDef htim2; // Primary PWM timer
extern TIM_HandleTypeDef htim3; // Secondary PWM timer (if needed)

// GPIO mapping for magnetorquer control (based on NUCLEO-L476RG pinout)
// H-Bridge enable pins (configure as GPIO outputs in CubeMX)
#define MAG_X_EN_GPIO_Port GPIOC
#define MAG_X_EN_Pin GPIO_PIN_0 // CN7 pin 35 (PC0)
#define MAG_Y_EN_GPIO_Port GPIOC
#define MAG_Y_EN_Pin GPIO_PIN_1 // CN7 pin 36 (PC1)
#define MAG_Z_EN_GPIO_Port GPIOC
#define MAG_Z_EN_Pin GPIO_PIN_2 // CN7 pin 37 (PC2)

// H-Bridge direction control pins
#define MAG_X_DIR_GPIO_Port GPIOC
#define MAG_X_DIR_Pin GPIO_PIN_3 // CN7 pin 38 (PC3)
#define MAG_Y_DIR_GPIO_Port GPIOB
#define MAG_Y_DIR_Pin GPIO_PIN_0 // CN8 pin 4 (PB0)
#define MAG_Z_DIR_GPIO_Port GPIOB
#define MAG_Z_DIR_Pin GPIO_PIN_1 // CN9 pin 23 (PB1)

// PWM timer channels (configure in CubeMX as PWM Generation)
#define MAG_X_TIM_CHANNEL TIM_CHANNEL_1 // TIM2_CH1 on PA0
#define MAG_Y_TIM_CHANNEL TIM_CHANNEL_2 // TIM2_CH2 on PA1
#define MAG_Z_TIM_CHANNEL TIM_CHANNEL_3 // TIM2_CH3 on PA2

// Hardware specifications based on Cornell Alpha implementation
#define MAG_WIRE_TURNS 100           // Number of wire turns per coil
#define MAG_COIL_AREA_M2 (25e-6f)    // Coil cross-sectional area (m²)
#define MAG_CORE_AMPLIFICATION 13.5f // Mu-metal core amplification factor
#define MAG_COIL_RESISTANCE 16.0f    // Coil resistance (Ohms)
#define MAG_MAX_CURRENT_A 0.25f      // Maximum current per coil (A)
#define MAG_SUPPLY_VOLTAGE 3.3f      // Nucleo supply voltage
#define MAG_PWM_MAX_VALUE 1000       // PWM counter maximum (0-1000)

// Control algorithm constants from Cornell implementation
#define MAG_CONTROL_FREQUENCY 10.0f // Control loop frequency (Hz)
#define MAG_PWM_FREQUENCY 1000.0f   // PWM switching frequency (Hz)

/**
 * @brief Magnetorquer axis enumeration
 */
typedef enum
{
    MAG_AXIS_X = 0,
    MAG_AXIS_Y = 1,
    MAG_AXIS_Z = 2,
    MAG_AXIS_COUNT = 3
} mag_axis_t;

/**
 * @brief Magnetorquer command structure
 */
typedef struct
{
    float current_a[MAG_AXIS_COUNT];     // Desired current per axis (A) [-0.25 to +0.25]
    float dipole_moment[MAG_AXIS_COUNT]; // Desired magnetic dipole (A⋅m²)
    uint16_t pwm_value[MAG_AXIS_COUNT];  // PWM duty cycle values (0-1000)
    uint8_t direction[MAG_AXIS_COUNT];   // Direction: 0=forward, 1=reverse
    bool enable[MAG_AXIS_COUNT];         // Enable/disable per axis
    uint32_t timestamp_ms;               // Command timestamp
    bool valid;                          // Command validity flag
} magnetorquer_cmd_t;

/**
 * @brief Magnetorquer calibration data (from Cornell testing)
 */
typedef struct
{
    float current_to_pwm_poly[MAG_AXIS_COUNT][4]; // Polynomial coefficients [a0,a1,a2,a3]
    float voltage_scale_factor;                   // Voltage-dependent scaling
    float voltage_offset;                         // Voltage-dependent offset
    float temp_compensation[MAG_AXIS_COUNT];      // Temperature compensation factors
    bool calibrated;                              // Calibration status
} magnetorquer_cal_t;

/**
 * @brief Magnetorquer device structure
 */
typedef struct
{
    // Hardware interface
    TIM_HandleTypeDef *htim_pwm;           // PWM timer handle
    uint32_t tim_channels[MAG_AXIS_COUNT]; // Timer channels per axis

    // Current command and status
    magnetorquer_cmd_t current_cmd;  // Current command
    magnetorquer_cmd_t previous_cmd; // Previous command for filtering

    // Calibration data
    magnetorquer_cal_t calibration; // Hardware calibration parameters

    // Safety and limits
    float max_current_per_axis; // Current limit per axis (A)
    float total_power_limit_w;  // Total power consumption limit (W)
    uint32_t last_update_ms;    // Last update timestamp

    // Status and diagnostics
    float estimated_power_w; // Current power consumption estimate
    uint8_t fault_flags;     // Fault status bitmask
    bool initialized;        // Initialization status
    bool enabled;            // Master enable/disable
} magnetorquer_t;

// Function prototypes (following your driver naming pattern)
/**
 * @brief Initialize magnetorquer system
 * @param dev Pointer to magnetorquer device structure
 * @param htim PWM timer handle
 * @return true on success, false on failure
 */
bool Magnetorquer_Init(magnetorquer_t *dev, TIM_HandleTypeDef *htim);

/**
 * @brief Set desired magnetic dipole moment
 * @param dev Pointer to device structure
 * @param dipole_x X-axis magnetic dipole (A⋅m²)
 * @param dipole_y Y-axis magnetic dipole (A⋅m²)
 * @param dipole_z Z-axis magnetic dipole (A⋅m²)
 * @return true on success, false on failure
 */
bool Magnetorquer_SetDipole(magnetorquer_t *dev, float dipole_x, float dipole_y, float dipole_z);

/**
 * @brief Set desired current per axis (alternative to dipole setting)
 * @param dev Pointer to device structure
 * @param current_a Array of 3 current values in Amperes
 * @return true on success, false on failure
 */
bool Magnetorquer_SetCurrent(magnetorquer_t *dev, const float current_a[MAG_AXIS_COUNT]);

/**
 * @brief Apply current command to hardware (updates PWM and GPIO)
 * @param dev Pointer to device structure
 * @return true on success, false on failure
 */
bool Magnetorquer_ApplyCommand(magnetorquer_t *dev);

/**
 * @brief Emergency stop - disable all magnetorquers immediately
 * @param dev Pointer to device structure
 */
void Magnetorquer_EmergencyStop(magnetorquer_t *dev);

/**
 * @brief Enable/disable magnetorquer system
 * @param dev Pointer to device structure
 * @param enable true to enable, false to disable
 * @return true on success, false on failure
 */
bool Magnetorquer_Enable(magnetorquer_t *dev, bool enable);

/**
 * @brief Load calibration parameters (from Cornell characterization)
 * @param dev Pointer to device structure
 * @param cal Pointer to calibration data structure
 * @return true on success, false on failure
 */
bool Magnetorquer_LoadCalibration(magnetorquer_t *dev, const magnetorquer_cal_t *cal);

/**
 * @brief Get current power consumption estimate
 * @param dev Pointer to device structure
 * @param power_w Pointer to store power estimate (Watts)
 * @return true on success, false on failure
 */
bool Magnetorquer_GetPowerEstimate(magnetorquer_t *dev, float *power_w);

/**
 * @brief Get current magnetorquer status
 * @param dev Pointer to device structure
 * @param cmd Pointer to store current command structure
 * @return true on success, false on failure
 */
bool Magnetorquer_GetStatus(magnetorquer_t *dev, magnetorquer_cmd_t *cmd);

/**
 * @brief Convert magnetic dipole moment to required current
 * @param dipole_moment Desired magnetic dipole (A⋅m²)
 * @return Required current (A)
 */
static inline float Magnetorquer_DipoleToCurrentModel(float dipole_moment)
{
    // Model: m = n * i * A * k (Cornell equation 8)
    return dipole_moment / (MAG_WIRE_TURNS * MAG_COIL_AREA_M2 * MAG_CORE_AMPLIFICATION);
}

/**
 * @brief Convert current to PWM value using Cornell calibration
 * @param dev Pointer to device structure
 * @param axis Axis index (0=X, 1=Y, 2=Z)
 * @param current_a Desired current (A)
 * @param battery_voltage Current battery/supply voltage (V)
 * @return PWM value (0-1000)
 */
uint16_t Magnetorquer_CurrentToPWM(magnetorquer_t *dev, mag_axis_t axis,
                                   float current_a, float battery_voltage);

#endif /* APPLICATION_DRIVERS_INC_MAGNETORQUER_H_ */
