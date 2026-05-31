#include "photodiode_runtime.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int nearly_equal(float a, float b, float eps)
{
    return fabsf(a - b) < eps;
}

static uint16_t raw_from_vector(const photodiode_calibration_t *cal, vec3_t sun_body)
{
    float projected = Vec3_Dot(cal->normal_body, sun_body);
    float raw = cal->dark_offset_counts;
    if (projected > 0.0f) {
        raw += projected * cal->gain_counts;
    }
    if (raw < 0.0f) {
        raw = 0.0f;
    }
    if (raw > 4095.0f) {
        raw = 4095.0f;
    }
    return (uint16_t)(raw + 0.5f);
}

static photodiode_sample_t sample_from_vector(vec3_t sun_body, uint8_t present_mask)
{
    photodiode_sample_t sample;
    uint32_t i;

    memset(&sample, 0, sizeof(sample));
    sample.present_mask = present_mask;
    sample.sample_timestamp_ms = 1000U;

    for (i = 0; i < PHOTODIODE_CHANNEL_COUNT; ++i) {
        photodiode_calibration_t cal;
        PhotodiodeRuntime_GetCalibration(i, &cal);
        sample.raw_counts[i] = raw_from_vector(&cal, sun_body);
    }

    return sample;
}

static void test_four_diodes_estimate_sun_vector(void)
{
    photodiode_sample_t sample;
    photodiode_snapshot_t snapshot;
    vec3_t sun_body = {0.0f, 0.0f, 1.0f};

    PhotodiodeRuntime_Init();
    sample = sample_from_vector(sun_body, 0x0FU);

    PhotodiodeRuntime_ProcessSample(&sample, &snapshot);

    assert(snapshot.sun_valid == 1U);
    assert(snapshot.degraded == 0U);
    assert(snapshot.used_mask == 0x0FU);
    assert(snapshot.valid_mask == 0x0FU);
    assert(snapshot.quality > 0.75f);
    assert(nearly_equal(snapshot.sun_body.x, 0.0f, 0.03f));
    assert(nearly_equal(snapshot.sun_body.y, 0.0f, 0.03f));
    assert(snapshot.sun_body.z > 0.99f);
}

static void test_one_failed_diode_still_solves_3d_vector(void)
{
    photodiode_sample_t sample;
    photodiode_snapshot_t snapshot;
    vec3_t sun_body = Vec3_Normalize((vec3_t){0.15f, -0.10f, 1.0f});

    PhotodiodeRuntime_Init();
    sample = sample_from_vector(sun_body, 0x0BU);

    PhotodiodeRuntime_ProcessSample(&sample, &snapshot);

    assert(snapshot.sun_valid == 1U);
    assert(snapshot.degraded == 0U);
    assert(snapshot.used_mask == 0x0BU);
    assert(nearly_equal(snapshot.sun_body.x, sun_body.x, 0.08f));
    assert(nearly_equal(snapshot.sun_body.y, sun_body.y, 0.08f));
    assert(snapshot.sun_body.z > 0.95f);
}

static void test_two_failed_diodes_keep_a_degraded_backup_vector(void)
{
    photodiode_sample_t sample;
    photodiode_snapshot_t snapshot;
    vec3_t first = {0.0f, 0.0f, 1.0f};
    vec3_t second = Vec3_Normalize((vec3_t){0.25f, 0.10f, 1.0f});

    PhotodiodeRuntime_Init();
    sample = sample_from_vector(first, 0x0FU);
    PhotodiodeRuntime_ProcessSample(&sample, &snapshot);
    assert(snapshot.sun_valid == 1U);

    sample = sample_from_vector(second, 0x03U);
    sample.sample_timestamp_ms = 1100U;
    PhotodiodeRuntime_ProcessSample(&sample, &snapshot);

    assert(snapshot.sun_valid == 1U);
    assert(snapshot.degraded == 1U);
    assert(snapshot.used_mask == 0x03U);
    assert(snapshot.quality < 0.75f);
    assert(Vec3_Norm(snapshot.sun_body) > 0.99f);
}

static void test_custom_normals_are_configurable(void)
{
    photodiode_calibration_t cal;
    photodiode_calibration_t out;

    PhotodiodeRuntime_Init();
    memset(&cal, 0, sizeof(cal));
    cal.enabled = 1U;
    cal.normal_body = Vec3_Normalize((vec3_t){1.0f, 1.0f, 1.0f});
    cal.dark_offset_counts = 11.0f;
    cal.gain_counts = 3000.0f;
    cal.min_valid_counts = 20.0f;
    cal.saturation_counts = 4090.0f;

    assert(PhotodiodeRuntime_SetCalibration(2U, &cal) == 1);
    assert(PhotodiodeRuntime_GetCalibration(2U, &out) == 1);
    assert(nearly_equal(out.normal_body.x, cal.normal_body.x, 1e-6f));
    assert(nearly_equal(out.dark_offset_counts, 11.0f, 1e-6f));
}

int main(void)
{
    test_four_diodes_estimate_sun_vector();
    test_one_failed_diode_still_solves_3d_vector();
    test_two_failed_diodes_keep_a_degraded_backup_vector();
    test_custom_normals_are_configurable();
    puts("photodiode_runtime_test: PASS");
    return 0;
}
