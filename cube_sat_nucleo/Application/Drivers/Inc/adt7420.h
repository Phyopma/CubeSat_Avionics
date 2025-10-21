#ifndef ADT7420_H
#define ADT7420_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

#define ADT7420_REG_TEMP_MSB   0x00
#define ADT7420_REG_TEMP_LSB   0x01
#define ADT7420_REG_STATUS     0x02
#define ADT7420_REG_CONFIG     0x03
#define ADT7420_REG_ID         0x0B
#define ADT7420_ID_EXPECTED    0xCB

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr7;   // 0x48..0x4B per JP2/JP1
} ADT7420_Handle;

HAL_StatusTypeDef ADT7420_Init(ADT7420_Handle *h, I2C_HandleTypeDef *hi2c, uint8_t addr7);
HAL_StatusTypeDef ADT7420_ReadID(ADT7420_Handle *h, uint8_t *id);
HAL_StatusTypeDef ADT7420_ReadConfig(ADT7420_Handle *h, uint8_t *cfg);
HAL_StatusTypeDef ADT7420_Set16Bit(ADT7420_Handle *h, uint8_t enable);
HAL_StatusTypeDef ADT7420_ReadRaw(ADT7420_Handle *h, int16_t *raw);
HAL_StatusTypeDef ADT7420_ReadCelsius(ADT7420_Handle *h, float *temp_c);

#endif
