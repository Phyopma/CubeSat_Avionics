#include "photodiode_runtime.h"

#include <math.h>
#include <string.h>

#if defined(USE_HAL_DRIVER)
#include "FreeRTOS.h"
#include "task.h"
#define PHOTODIODE_ENTER_CRITICAL() taskENTER_CRITICAL()
#define PHOTODIODE_EXIT_CRITICAL()  taskEXIT_CRITICAL()
#else
#define PHOTODIODE_ENTER_CRITICAL() ((void)0)
#define PHOTODIODE_EXIT_CRITICAL()  ((void)0)
#endif

#define PHOTODIODE_DEFAULT_DARK_COUNTS 20.0f
#define PHOTODIODE_DEFAULT_GAIN_COUNTS 3000.0f
#define PHOTODIODE_DEFAULT_MIN_COUNTS  25.0f
#define PHOTODIODE_DEFAULT_SAT_COUNTS  4090.0f
#define PHOTODIODE_HOLD_MS             500U
#define PHOTODIODE_DET_EPS            1.0e-5f

typedef struct {
    photodiode_calibration_t calibration[PHOTODIODE_CHANNEL_COUNT];
    photodiode_snapshot_t snapshots[2];
    uint8_t published_index;
    uint32_t publish_sequence;
    vec3_t last_sun_body;
    uint32_t last_sun_timestamp_ms;
    uint8_t last_sun_valid;
} photodiode_runtime_state_t;

static photodiode_runtime_state_t g_photodiode_runtime;

static vec3_t normalize_or_default(vec3_t v, vec3_t fallback)
{
    float n = Vec3_Norm(v);
    if (n < 1.0e-6f) {
        return fallback;
    }
    return Vec3_ScalarMult(v, 1.0f / n);
}

static photodiode_calibration_t default_calibration(uint32_t channel)
{
    static const vec3_t default_normals[PHOTODIODE_CHANNEL_COUNT] = {
        { 0.30f,  0.30f, 1.0f},
        {-0.30f,  0.30f, 1.0f},
        {-0.30f, -0.30f, 1.0f},
        { 0.30f, -0.30f, 1.0f}
    };
    photodiode_calibration_t cal;

    cal.enabled = 1U;
    cal.normal_body = normalize_or_default(default_normals[channel], (vec3_t){0.0f, 0.0f, 1.0f});
    cal.dark_offset_counts = PHOTODIODE_DEFAULT_DARK_COUNTS;
    cal.gain_counts = PHOTODIODE_DEFAULT_GAIN_COUNTS;
    cal.min_valid_counts = PHOTODIODE_DEFAULT_MIN_COUNTS;
    cal.saturation_counts = PHOTODIODE_DEFAULT_SAT_COUNTS;
    return cal;
}

static uint8_t popcount4(uint8_t mask)
{
    uint8_t count = 0U;
    uint32_t i;
    for (i = 0U; i < PHOTODIODE_CHANNEL_COUNT; ++i) {
        if ((mask & (uint8_t)(1U << i)) != 0U) {
            count++;
        }
    }
    return count;
}

static int solve_3x3(float a[3][3], float b[3], vec3_t *out)
{
    float det =
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
        a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
        a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if (fabsf(det) < PHOTODIODE_DET_EPS || out == NULL) {
        return 0;
    }

    out->x = (
        b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
        a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2]) +
        a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2])
    ) / det;

    out->y = (
        a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2]) -
        b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
        a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0])
    ) / det;

    out->z = (
        a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1]) -
        a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0]) +
        b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    ) / det;

    return Vec3_Norm(*out) > 1.0e-6f;
}

static vec3_t weighted_normal_fallback(const photodiode_snapshot_t *snapshot)
{
    vec3_t accum = {0.0f, 0.0f, 0.0f};
    uint32_t i;

    for (i = 0U; i < PHOTODIODE_CHANNEL_COUNT; ++i) {
        uint8_t bit = (uint8_t)(1U << i);
        if ((snapshot->used_mask & bit) == 0U) {
            continue;
        }
        accum = Vec3_Add(accum, Vec3_ScalarMult(g_photodiode_runtime.calibration[i].normal_body,
                                                snapshot->calibrated[i]));
    }

    return normalize_or_default(accum, (vec3_t){0.0f, 0.0f, 1.0f});
}

void PhotodiodeRuntime_Init(void)
{
    uint32_t i;
    (void)memset(&g_photodiode_runtime, 0, sizeof(g_photodiode_runtime));
    for (i = 0U; i < PHOTODIODE_CHANNEL_COUNT; ++i) {
        g_photodiode_runtime.calibration[i] = default_calibration(i);
    }
    g_photodiode_runtime.last_sun_body = (vec3_t){0.0f, 0.0f, 1.0f};
}

int PhotodiodeRuntime_SetCalibration(uint32_t channel, const photodiode_calibration_t *calibration)
{
    photodiode_calibration_t next;
    if (channel >= PHOTODIODE_CHANNEL_COUNT || calibration == NULL) {
        return 0;
    }

    next = *calibration;
    next.normal_body = normalize_or_default(next.normal_body, default_calibration(channel).normal_body);
    if (next.gain_counts < 1.0f) {
        next.gain_counts = PHOTODIODE_DEFAULT_GAIN_COUNTS;
    }
    if (next.saturation_counts <= next.dark_offset_counts) {
        next.saturation_counts = PHOTODIODE_DEFAULT_SAT_COUNTS;
    }

    PHOTODIODE_ENTER_CRITICAL();
    g_photodiode_runtime.calibration[channel] = next;
    PHOTODIODE_EXIT_CRITICAL();
    return 1;
}

int PhotodiodeRuntime_GetCalibration(uint32_t channel, photodiode_calibration_t *out)
{
    if (channel >= PHOTODIODE_CHANNEL_COUNT || out == NULL) {
        return 0;
    }

    PHOTODIODE_ENTER_CRITICAL();
    *out = g_photodiode_runtime.calibration[channel];
    PHOTODIODE_EXIT_CRITICAL();
    return 1;
}

void PhotodiodeRuntime_ProcessSample(const photodiode_sample_t *sample, photodiode_snapshot_t *out)
{
    photodiode_snapshot_t snapshot;
    float normal_eq[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    float rhs[3] = {0.0f, 0.0f, 0.0f};
    uint8_t valid_count;
    uint32_t i;
    vec3_t solved = {0.0f, 0.0f, 0.0f};
    uint8_t solved_ok = 0U;

    (void)memset(&snapshot, 0, sizeof(snapshot));
    if (sample == NULL) {
        if (out != NULL) {
            *out = snapshot;
        }
        return;
    }

    snapshot.publish_timestamp_ms = sample->sample_timestamp_ms;
    for (i = 0U; i < PHOTODIODE_CHANNEL_COUNT; ++i) {
        uint8_t bit = (uint8_t)(1U << i);
        const photodiode_calibration_t *cal = &g_photodiode_runtime.calibration[i];
        float signal;

        snapshot.raw_counts[i] = sample->raw_counts[i];
        if ((sample->present_mask & bit) == 0U || cal->enabled == 0U) {
            snapshot.failed_mask |= bit;
            continue;
        }

        signal = (float)sample->raw_counts[i] - cal->dark_offset_counts;
        if (signal < 0.0f) {
            signal = 0.0f;
        }
        snapshot.calibrated[i] = signal / cal->gain_counts;
        if (snapshot.calibrated[i] > 1.0f) {
            snapshot.calibrated[i] = 1.0f;
        }
        if ((float)sample->raw_counts[i] >= cal->saturation_counts) {
            snapshot.saturated_mask |= bit;
            continue;
        }
        if (signal < cal->min_valid_counts) {
            snapshot.failed_mask |= bit;
            continue;
        }

        snapshot.valid_mask |= bit;
        snapshot.used_mask |= bit;
    }

    valid_count = popcount4(snapshot.used_mask);
    if (valid_count >= 3U) {
        for (i = 0U; i < PHOTODIODE_CHANNEL_COUNT; ++i) {
            uint8_t bit = (uint8_t)(1U << i);
            vec3_t n;
            float m;
            if ((snapshot.used_mask & bit) == 0U) {
                continue;
            }
            n = g_photodiode_runtime.calibration[i].normal_body;
            m = snapshot.calibrated[i];
            normal_eq[0][0] += n.x * n.x;
            normal_eq[0][1] += n.x * n.y;
            normal_eq[0][2] += n.x * n.z;
            normal_eq[1][0] += n.y * n.x;
            normal_eq[1][1] += n.y * n.y;
            normal_eq[1][2] += n.y * n.z;
            normal_eq[2][0] += n.z * n.x;
            normal_eq[2][1] += n.z * n.y;
            normal_eq[2][2] += n.z * n.z;
            rhs[0] += n.x * m;
            rhs[1] += n.y * m;
            rhs[2] += n.z * m;
        }
        solved_ok = (uint8_t)solve_3x3(normal_eq, rhs, &solved);
    }

    if (solved_ok != 0U) {
        snapshot.sun_body = Vec3_Normalize(solved);
        snapshot.sun_valid = 1U;
        snapshot.degraded = 0U;
        snapshot.quality = (float)valid_count / (float)PHOTODIODE_CHANNEL_COUNT;
    } else if (valid_count >= 2U) {
        snapshot.sun_body = weighted_normal_fallback(&snapshot);
        snapshot.sun_valid = 1U;
        snapshot.degraded = 1U;
        snapshot.quality = 0.45f + (0.10f * (float)(valid_count - 2U));
    } else if (valid_count == 1U) {
        uint8_t hold_valid = 0U;
        if (g_photodiode_runtime.last_sun_valid != 0U &&
            (uint32_t)(sample->sample_timestamp_ms - g_photodiode_runtime.last_sun_timestamp_ms) <= PHOTODIODE_HOLD_MS) {
            snapshot.sun_body = g_photodiode_runtime.last_sun_body;
            hold_valid = 1U;
        } else {
            snapshot.sun_body = weighted_normal_fallback(&snapshot);
        }
        snapshot.sun_valid = 1U;
        snapshot.degraded = 1U;
        snapshot.quality = hold_valid ? 0.35f : 0.20f;
    } else if (g_photodiode_runtime.last_sun_valid != 0U &&
               (uint32_t)(sample->sample_timestamp_ms - g_photodiode_runtime.last_sun_timestamp_ms) <= PHOTODIODE_HOLD_MS) {
        snapshot.sun_body = g_photodiode_runtime.last_sun_body;
        snapshot.sun_valid = 1U;
        snapshot.degraded = 1U;
        snapshot.quality = 0.15f;
    }

    if (snapshot.sun_valid != 0U) {
        g_photodiode_runtime.last_sun_body = snapshot.sun_body;
        g_photodiode_runtime.last_sun_timestamp_ms = sample->sample_timestamp_ms;
        g_photodiode_runtime.last_sun_valid = 1U;
    }

    PhotodiodeRuntime_PublishSnapshot(&snapshot);
    if (out != NULL) {
        *out = snapshot;
    }
}

void PhotodiodeRuntime_PublishSnapshot(const photodiode_snapshot_t *snapshot)
{
    uint8_t next_index;
    if (snapshot == NULL) {
        return;
    }

    PHOTODIODE_ENTER_CRITICAL();
    next_index = (uint8_t)(g_photodiode_runtime.published_index ^ 1U);
    g_photodiode_runtime.snapshots[next_index] = *snapshot;
    g_photodiode_runtime.publish_sequence += 1U;
    g_photodiode_runtime.snapshots[next_index].publish_sequence = g_photodiode_runtime.publish_sequence;
    g_photodiode_runtime.published_index = next_index;
    PHOTODIODE_EXIT_CRITICAL();
}

void PhotodiodeRuntime_ReadSnapshot(photodiode_snapshot_t *out)
{
    if (out == NULL) {
        return;
    }

    PHOTODIODE_ENTER_CRITICAL();
    *out = g_photodiode_runtime.snapshots[g_photodiode_runtime.published_index];
    PHOTODIODE_EXIT_CRITICAL();
}
