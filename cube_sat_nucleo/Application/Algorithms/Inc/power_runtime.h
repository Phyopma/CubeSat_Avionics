#ifndef POWER_RUNTIME_H
#define POWER_RUNTIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POWER_BATTERY_CELL_COUNT 4U
#define POWER_BATTERY_TEMP_COUNT 8U

#define POWER_BATTERY_VALID_CELL_VOLTAGE_1   (1UL << 0)
#define POWER_BATTERY_VALID_CELL_VOLTAGE_2   (1UL << 1)
#define POWER_BATTERY_VALID_CELL_VOLTAGE_3   (1UL << 2)
#define POWER_BATTERY_VALID_CELL_VOLTAGE_4   (1UL << 3)
#define POWER_BATTERY_VALID_CURRENT          (1UL << 4)
#define POWER_BATTERY_VALID_AVERAGE_CURRENT  (1UL << 5)
#define POWER_BATTERY_VALID_VOLTAGE          (1UL << 6)
#define POWER_BATTERY_VALID_AVERAGE_VOLTAGE  (1UL << 7)
#define POWER_BATTERY_VALID_CYCLE_COUNT      (1UL << 8)
#define POWER_BATTERY_VALID_TBAT             (1UL << 9)
#define POWER_BATTERY_VALID_CELL_TEMP_1      (1UL << 10)
#define POWER_BATTERY_VALID_CELL_TEMP_2      (1UL << 11)
#define POWER_BATTERY_VALID_CELL_TEMP_3      (1UL << 12)
#define POWER_BATTERY_VALID_CELL_TEMP_4      (1UL << 13)
#define POWER_BATTERY_VALID_CELL_TEMP_5      (1UL << 14)
#define POWER_BATTERY_VALID_CELL_TEMP_6      (1UL << 15)
#define POWER_BATTERY_VALID_CELL_TEMP_7      (1UL << 16)
#define POWER_BATTERY_VALID_CELL_TEMP_8      (1UL << 17)

#define POWER_DIGITAL_TEMP_VALID_TEMPERATURE (1U << 0)
#define POWER_DIGITAL_TEMP_VALID_STATUS      (1U << 1)

typedef struct {
    uint32_t valid_mask;
    uint16_t cell_voltage_mV[POWER_BATTERY_CELL_COUNT];
    int16_t current_mA;
    int16_t average_current_mA;
    uint16_t voltage_mV;
    uint16_t average_voltage_mV;
    uint16_t cycle_count;
    uint16_t battery_temperature_K;
    uint16_t cell_temperature_dK[POWER_BATTERY_TEMP_COUNT];
} power_battery_data_t;

typedef struct {
    uint8_t valid_mask;
    int16_t temperature_cC;
    uint8_t status;
} power_digital_temp_data_t;

typedef struct {
    uint32_t publish_sequence;
    uint32_t publish_timestamp_ms;
    uint32_t source_age_ms;
    power_battery_data_t battery;
    power_digital_temp_data_t digital_temp;
} power_snapshot_t;

void Power_Init(void);
void Power_PublishSnapshot(const power_snapshot_t *snapshot);
void Power_ReadSnapshot(power_snapshot_t *out);

#ifdef __cplusplus
}
#endif

#endif
