#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <stdint.h>

// Init function
void CurrentSensor_Init(void);

// Returns current in Amps
float CurrentSensor_Read_Amps(void);

#endif