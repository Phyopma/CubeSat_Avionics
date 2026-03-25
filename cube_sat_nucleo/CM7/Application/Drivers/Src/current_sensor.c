#include "current_sensor.h"

typedef struct {
    volatile uint8_t request_pending;
    volatile uint8_t transfer_in_progress;
    volatile uint8_t sample_valid;
    volatile uint32_t last_sample_tick_ms;
    volatile float latest_current_amps;

    uint8_t raw_buf[2];
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

void CurrentSensor_Init(void)
{
	// 1. Reset INA219
	write_reg(REG_CONFIG, 0x8000);
	// Keep init non-blocking before scheduler start.

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

float CurrentSensor_ConvertRawToAmps(int16_t raw_value)
{
	// Convert to Amps
	// LSB = 0.1mA -> divide by 10 to get mA, divide by 10000 to get Amps
	// simpler: raw * 0.0001
	return (float)raw_value * 0.0001f;
}

void CurrentSensor_SubmitSampleRequest(void)
{
    g_async_state.request_pending = 1U;
}

/*
 * Function will process samples and return data on
 * hi2c1 line. Data will show up async and
 * processed by the callback function.
 */
void CurrentSensor_ProcessSample(void)
{
    if (g_async_state.request_pending == 0U) {
        return;
    }

    if (g_async_state.transfer_in_progress != 0U) {
    	return;
    }

    g_async_state.request_pending = 0U;
    g_async_state.transfer_in_progress = 1U;

    /*
     * HAL_I2C_Mem_Read_IT is used for async i2c transfer.\
     * Callback function will process data
     */
    if (HAL_I2C_Mem_Read_IT(&hi2c1, INA219_ADDR, &reg_addr, I2C_MEMADD_SIZE_8BIT, buffer, 2) != HAL_OK)
    {
    	g_async_state.transfer_in_progress = 0U; // i2c transfer did not start
    }
}

/*
 * This is the I2C callback and will run when data is read on the line.
 * Note that this function assumes the only data on hi2c1 is data from
 * the current sensor.
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c != &hi2c1)
	{
		return;
	}

    g_async_state.transfer_in_progress = 0U;
    g_async_state.sample_ready = 1U;
    g_async_state.last_sample_tick_ms = HAL_GetTick();

    int16_t raw_current =
           (int16_t)(((uint16_t)g_async_state.raw_buf[0] << 8) |
                     ((uint16_t)g_async_state.raw_buf[1]));

    g_async_state.latest_current_amps = CurrentSensor_ConvertRawToAmps(raw_current);
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
