#ifndef PHOTODIODE_RUNTIME_H
#define PHOTODIODE_RUNTIME_H

#include "math_lib.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PHOTODIODE_CHANNEL_COUNT 4U

typedef struct {
    uint8_t enabled;
    vec3_t normal_body;
    float dark_offset_counts;
    float gain_counts;
    float min_valid_counts;
    float saturation_counts;
} photodiode_calibration_t;

typedef struct {
    uint16_t raw_counts[PHOTODIODE_CHANNEL_COUNT];
    uint8_t present_mask;
    uint32_t sample_timestamp_ms;
} photodiode_sample_t;

typedef struct {
    uint32_t publish_sequence;
    uint32_t publish_timestamp_ms;
    uint16_t raw_counts[PHOTODIODE_CHANNEL_COUNT];
    float calibrated[PHOTODIODE_CHANNEL_COUNT];
    uint8_t valid_mask;
    uint8_t saturated_mask;
    uint8_t failed_mask;
    uint8_t used_mask;
    uint8_t sun_valid;
    uint8_t degraded;
    float quality;
    vec3_t sun_body;
} photodiode_snapshot_t;

void PhotodiodeRuntime_Init(void);
int PhotodiodeRuntime_SetCalibration(uint32_t channel, const photodiode_calibration_t *calibration);
int PhotodiodeRuntime_GetCalibration(uint32_t channel, photodiode_calibration_t *out);
void PhotodiodeRuntime_ProcessSample(const photodiode_sample_t *sample, photodiode_snapshot_t *out);
void PhotodiodeRuntime_PublishSnapshot(const photodiode_snapshot_t *snapshot);
void PhotodiodeRuntime_ReadSnapshot(photodiode_snapshot_t *out);

#ifdef __cplusplus
}
#endif

#endif
