#include "photodiode.h"

#include <string.h>

/*
 * Photodiode ADC scaffold for the H755ZI CM7 port.
 *
 * The current CubeMX project has no ADC peripheral generated yet. Until the
 * final solar-panel photodiode board and connector are chosen, this driver
 * publishes "not present" samples and lets photodiode_runtime keep the ADCS
 * Sun target invalid.
 *
 * Suggested NUCLEO-H755ZI-Q ADC candidates that do not collide with the
 * current .ioc assignments:
 *   D0: PA6  / ADC12_INP3   / CN7-8   (verify board header before wiring)
 *   D1: PC0  / ADC123_INP10 / D32-A10 (verify board header before wiring)
 *   D2: PC1  / ADC123_INP11 / D33-A11 (verify board header before wiring)
 *   D3: PC2  / ADC123_INP12 / D34-A12 (verify board header before wiring)
 *
 * When CubeMX enables ADC1 or ADC2, replace this placeholder with the standard
 * HAL sequence documented by STM32CubeH7 HAL:
 *   HAL_ADC_Start(&hadcX);
 *   HAL_ADC_PollForConversion(&hadcX, timeout);
 *   HAL_ADC_GetValue(&hadcX);
 *   HAL_ADC_Stop(&hadcX);
 *
 * If an external 4-channel ADC is selected instead, keep this public interface
 * and move the bus-specific I2C/SPI reads inside Photodiode_ReadRaw().
 */

void Photodiode_Init(void)
{
}

int Photodiode_ReadRaw(photodiode_sample_t *out, uint32_t now_ms)
{
    if (out == NULL) {
        return 0;
    }

    (void)memset(out, 0, sizeof(*out));
    out->sample_timestamp_ms = now_ms;
    out->present_mask = 0U;
    return 0;
}
