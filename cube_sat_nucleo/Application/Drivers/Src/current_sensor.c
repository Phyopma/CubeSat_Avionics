#include "current_sensor.h"

void CurrentSensor_Init(void)
{
    // FUTURE: Initialize I2C1 and configure INA219 here.
}

float CurrentSensor_Read_Amps(void)
{
    // FUTURE: Read I2C register, convert to float.

    // CURRENT: Return 0.0 since hardware is missing.
    return 0.0f;
}