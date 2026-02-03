#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <stdint.h>
#include "main.h"

#define INA219_ADDR (0x40 << 1)

// Registers
#define REG_CONFIG      0x00
#define REG_SHUNT_VOLT  0x01
#define REG_BUS_VOLT    0x02
#define REG_POWER       0x03
#define REG_CURRENT     0x04
#define REG_CALIBRATION 0x05


extern I2C_HandleTypeDef hi2c1;

// Init function
void CurrentSensor_Init(void);

// Returns current in Amps
float CurrentSensor_Read_Amps(void);

#endif
