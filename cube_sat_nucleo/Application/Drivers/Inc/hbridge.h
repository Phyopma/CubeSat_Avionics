/* Application/Drivers/Inc/hbridge.h */
#ifndef HBRIDGE_H
#define HBRIDGE_H

#include "stm32l4xx_hal.h"
#include <stdbool.h>

// --- Function Prototypes ---

/**
 * @brief Initializes PWM channels and enables the H-Bridge driver (Sleep pin HIGH).
 */
void HBridge_Init(void);

/**
 * @brief Sets the H-Bridge output voltage via PWM duty cycle.
 * @param axis: 0=X, 1=Y, 2=Z
 * @param voltage_volts: Desired voltage (can be positive or negative).
 * @param max_supply_volts: The actual battery voltage (VCC) supplying the bridge.
 */
void HBridge_SetVoltage(uint8_t axis, float voltage_volts, float max_supply_volts);

/**
 * @brief Puts the H-Bridge into low-power sleep mode.
 * @param enable_sleep: true = Sleep (OFF), false = Wake (ON)
 */
void HBridge_Sleep(bool enable_sleep);

#endif /* HBRIDGE_H */