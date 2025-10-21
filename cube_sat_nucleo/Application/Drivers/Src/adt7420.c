#include "adt7420.h"

static HAL_StatusTypeDef wr_u8(ADT7420_Handle *h, uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(h->hi2c, (h->addr7 << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
}
static HAL_StatusTypeDef rd_u8(ADT7420_Handle *h, uint8_t reg, uint8_t *val) {
    return HAL_I2C_Mem_Read(h->hi2c, (h->addr7 << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, val, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADT7420_ReadID(ADT7420_Handle *h, uint8_t *id) {
    return rd_u8(h, ADT7420_REG_ID, id);
}
HAL_StatusTypeDef ADT7420_ReadConfig(ADT7420_Handle *h, uint8_t *cfg) {
    return rd_u8(h, ADT7420_REG_CONFIG, cfg);
}
HAL_StatusTypeDef ADT7420_Set16Bit(ADT7420_Handle *h, uint8_t enable) {
    uint8_t cfg = 0;
    HAL_StatusTypeDef st = rd_u8(h, ADT7420_REG_CONFIG, &cfg);
    if (st != HAL_OK) return st;
    if (enable) cfg |=  (1u << 7);
    else        cfg &= ~(1u << 7);
    st = wr_u8(h, ADT7420_REG_CONFIG, cfg);
    if (st == HAL_OK) HAL_Delay(250);
    return st;
}
HAL_StatusTypeDef ADT7420_ReadRaw(ADT7420_Handle *h, int16_t *raw) {
    uint8_t buf[2];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(h->hi2c, (h->addr7 << 1),
                                            ADT7420_REG_TEMP_MSB,
                                            I2C_MEMADD_SIZE_8BIT,
                                            buf, 2, HAL_MAX_DELAY);
    if (st != HAL_OK) return st;
    *raw = (int16_t)((buf[0] << 8) | buf[1]);
    return HAL_OK;
}
HAL_StatusTypeDef ADT7420_ReadCelsius(ADT7420_Handle *h, float *temp_c) {
    uint8_t cfg = 0;
    int16_t raw = 0;
    HAL_StatusTypeDef st = ADT7420_ReadConfig(h, &cfg);
    if (st != HAL_OK) return st;
    st = ADT7420_ReadRaw(h, &raw);
    if (st != HAL_OK) return st;

    if (cfg & 0x80) *temp_c = ((float)raw) / 128.0f; // 16-bit mode
    else            *temp_c = ((float)(raw >> 3)) / 16.0f;   // 13-bit default
    return HAL_OK;
}
HAL_StatusTypeDef ADT7420_Init(ADT7420_Handle *h, I2C_HandleTypeDef *hi2c, uint8_t addr7) {
    h->hi2c = hi2c;
    h->addr7 = addr7;
    uint8_t id = 0;
    HAL_StatusTypeDef st = ADT7420_ReadID(h, &id);
    if (st != HAL_OK) return st;
    if (id != ADT7420_ID_EXPECTED) return HAL_ERROR;
    HAL_Delay(250);
    return HAL_OK;
}
