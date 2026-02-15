#include "current_sensor.h"

typedef struct {
    volatile uint8_t request_pending;
    volatile uint8_t sample_valid;
    volatile uint32_t last_sample_tick_ms;
    volatile float latest_current_amps;
} current_sensor_async_state_t;

static current_sensor_async_state_t g_async_state = {0};

// Helper: Write 16-bit Register
static void write_reg(uint8_t reg, uint16_t val) {
    uint8_t data[3];
    data[0] = reg;
    data[1] = (val >> 8) & 0xFF; // High Byte
    data[2] = val & 0xFF;        // Low Byte
    HAL_I2C_Master_Transmit(&hi2c1, INA219_ADDR, data, 3, 10);
}

// Helper: Read 16-bit Register
static uint16_t read_reg(uint8_t reg) {
    uint8_t reg_addr = reg;
    uint8_t buffer[2] = {0, 0};

    // 1. Send Register Address
    HAL_I2C_Master_Transmit(&hi2c1, INA219_ADDR, &reg_addr, 1, 10);
    // 2. Read 2 Bytes
    HAL_I2C_Master_Receive(&hi2c1, INA219_ADDR, buffer, 2, 10);

    return (uint16_t)((buffer[0] << 8) | buffer[1]);
}


void CurrentSensor_Init(void)
{
	// 1. Reset INA219
	write_reg(REG_CONFIG, 0x8000);
	HAL_Delay(10);

	// 2. Calibrate
	// Value 4096 is standard for 0.1 Ohm shunt, 32V range, 2A max
	// This makes the Current Register LSB = 0.1mA
	write_reg(REG_CALIBRATION, 4096);

	// 3. Configure
	// Range: 32V, Gain: /8 (320mV), ADC: 12-bit + 8x Avg (Filter)
	// Mode: Continuous Shunt + Bus
	// Value: 0x399F
	write_reg(REG_CONFIG, 0x399F);
    g_async_state.request_pending = 0U;
    g_async_state.sample_valid = 0U;
    g_async_state.last_sample_tick_ms = 0U;
    g_async_state.latest_current_amps = 0.0f;
}

float CurrentSensor_Read_Amps(void)
{
	// Read Raw signed 16-bit value
	int16_t raw = (int16_t)read_reg(REG_CURRENT);

	// Convert to Amps
	// LSB = 0.1mA -> divide by 10 to get mA, divide by 10000 to get Amps
	// simpler: raw * 0.0001
	return (float)raw * 0.0001f;
}

void CurrentSensor_SubmitSampleRequest(void)
{
    g_async_state.request_pending = 1U;
}

void CurrentSensor_RunAsyncSample(void)
{
    if (g_async_state.request_pending == 0U) {
        return;
    }
    g_async_state.request_pending = 0U;
    g_async_state.latest_current_amps = CurrentSensor_Read_Amps();
    g_async_state.last_sample_tick_ms = HAL_GetTick();
    g_async_state.sample_valid = 1U;
}

int CurrentSensor_GetLatestSample(float *amps, uint32_t *age_ms, uint32_t now_ms)
{
    if (g_async_state.sample_valid == 0U) {
        return 0;
    }
    if (amps != NULL) {
        *amps = g_async_state.latest_current_amps;
    }
    if (age_ms != NULL) {
        *age_ms = now_ms - g_async_state.last_sample_tick_ms;
    }
    return 1;
}
