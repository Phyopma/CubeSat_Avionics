#include "imu_runtime.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_request_arbitration_uses_union_and_fastest_interval(void)
{
    imu_request_set_t control = {0};
    imu_request_set_t comms = {0};
    imu_request_set_t resolved = {0};

    IMU_Init();

    control.reports[IMU_REPORT_GYRO].enabled = 1U;
    control.reports[IMU_REPORT_GYRO].interval_us = 50000U;
    control.reports[IMU_REPORT_MAG_CAL].enabled = 1U;
    control.reports[IMU_REPORT_MAG_CAL].interval_us = 50000U;

    comms.reports[IMU_REPORT_GYRO].enabled = 1U;
    comms.reports[IMU_REPORT_GYRO].interval_us = 100000U;
    comms.reports[IMU_REPORT_ROTATION_VECTOR].enabled = 1U;
    comms.reports[IMU_REPORT_ROTATION_VECTOR].interval_us = 100000U;

    IMU_RequestSet(IMU_CLIENT_CONTROL, &control);
    IMU_RequestSet(IMU_CLIENT_COMMS, &comms);
    IMU_CopyResolvedRequestSet(&resolved);

    assert(resolved.reports[IMU_REPORT_GYRO].enabled == 1U);
    assert(resolved.reports[IMU_REPORT_GYRO].interval_us == 50000U);
    assert(resolved.reports[IMU_REPORT_MAG_CAL].enabled == 1U);
    assert(resolved.reports[IMU_REPORT_ROTATION_VECTOR].enabled == 1U);
}

static void test_raw_mag_dependency_forces_calibrated_mag(void)
{
    imu_request_set_t control = {0};
    imu_request_set_t resolved = {0};

    IMU_Init();

    control.reports[IMU_REPORT_MAG_RAW].enabled = 1U;
    control.reports[IMU_REPORT_MAG_RAW].interval_us = 50000U;

    IMU_RequestSet(IMU_CLIENT_CONTROL, &control);
    IMU_CopyResolvedRequestSet(&resolved);

    assert(resolved.reports[IMU_REPORT_MAG_RAW].enabled == 1U);
    assert(resolved.reports[IMU_REPORT_MAG_CAL].enabled == 1U);
    assert(resolved.reports[IMU_REPORT_MAG_CAL].interval_us == 50000U);
}

static void test_control_mode_profile_adds_unfused_mag_when_mtq_is_firing(void)
{
    imu_request_set_t control = {0};

    IMU_BuildControlRequestSet(CTRL_MODE_POINTING, true, &control);

    assert(control.reports[IMU_REPORT_GYRO].enabled == 1U);
    assert(control.reports[IMU_REPORT_ROTATION_VECTOR].enabled == 1U);
    assert(control.reports[IMU_REPORT_GAME_ROTATION_VECTOR].enabled == 1U);
    assert(control.reports[IMU_REPORT_MAG_CAL].enabled == 1U);
    assert(control.reports[IMU_REPORT_MAG_UNCAL].enabled == 1U);
    assert(control.reports[IMU_REPORT_MAG_RAW].enabled == 1U);
}

static void test_snapshot_publish_and_read_are_stable(void)
{
    imu_snapshot_t snapshot_a;
    imu_snapshot_t snapshot_b;
    imu_snapshot_t out;

    IMU_Init();

    (void)memset(&snapshot_a, 0, sizeof(snapshot_a));
    (void)memset(&snapshot_b, 0, sizeof(snapshot_b));

    snapshot_a.publish_timestamp_ms = 111U;
    snapshot_a.gyro.meta.valid = 1U;
    snapshot_a.gyro.value.x = 1.0f;

    snapshot_b.publish_timestamp_ms = 222U;
    snapshot_b.gyro.meta.valid = 1U;
    snapshot_b.gyro.value.x = 2.0f;
    snapshot_b.magnetometer_raw.meta.valid = 1U;
    snapshot_b.magnetometer_raw.value.z = -42;

    IMU_PublishSnapshot(&snapshot_a);
    IMU_ReadSnapshot(&out);
    assert(out.publish_sequence == 1U);
    assert(out.publish_timestamp_ms == 111U);
    assert(out.gyro.value.x == 1.0f);

    IMU_PublishSnapshot(&snapshot_b);
    IMU_ReadSnapshot(&out);
    assert(out.publish_sequence == 2U);
    assert(out.publish_timestamp_ms == 222U);
    assert(out.gyro.value.x == 2.0f);
    assert(out.magnetometer_raw.value.z == -42);
}

int main(void)
{
    test_request_arbitration_uses_union_and_fastest_interval();
    test_raw_mag_dependency_forces_calibrated_mag();
    test_control_mode_profile_adds_unfused_mag_when_mtq_is_firing();
    test_snapshot_publish_and_read_are_stable();
    puts("imu_runtime_test: PASS");
    return 0;
}
