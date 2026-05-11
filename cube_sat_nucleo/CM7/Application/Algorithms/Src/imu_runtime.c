#include "imu_runtime.h"

#include "config.h"
#include <string.h>

#if defined(USE_HAL_DRIVER)
#include "FreeRTOS.h"
#include "task.h"
#define IMU_RUNTIME_ENTER_CRITICAL() taskENTER_CRITICAL()
#define IMU_RUNTIME_EXIT_CRITICAL()  taskEXIT_CRITICAL()
#else
#define IMU_RUNTIME_ENTER_CRITICAL() ((void)0)
#define IMU_RUNTIME_EXIT_CRITICAL()  ((void)0)
#endif

#define IMU_FEATURE_ROTATION_VECTOR      0x05U
#define IMU_FEATURE_GAME_ROTATION_VECTOR 0x08U
#define IMU_FEATURE_GYROSCOPE            0x02U
#define IMU_FEATURE_MAGNETIC_FIELD       0x03U
#define IMU_FEATURE_MAGNETIC_FIELD_UNCAL 0x0FU
#define IMU_FEATURE_RAW_MAGNETOMETER     0x16U
#define IMU_FEATURE_LINEAR_ACCELERATION  0x04U
#define IMU_FEATURE_ACCELEROMETER        0x01U
#define IMU_FEATURE_GRAVITY              0x06U

typedef struct {
    imu_request_set_t client_requests[IMU_CLIENT_COUNT];
    imu_request_set_t resolved_request_set;
    imu_request_set_t applied_request_set;
    imu_snapshot_t snapshots[2];
    uint8_t published_index;
    uint32_t publish_sequence;
} imu_runtime_state_t;

static imu_runtime_state_t g_imu_runtime;

static void IMU_ClearRequestSet(imu_request_set_t *set)
{
    if (set == NULL) {
        return;
    }
    (void)memset(set, 0, sizeof(*set));
}

static void IMU_SetRequest(imu_request_set_t *set, imu_report_id_t report_id, uint32_t interval_us)
{
    if (set == NULL || report_id >= IMU_REPORT_COUNT) {
        return;
    }
    set->reports[report_id].enabled = 1U;
    set->reports[report_id].interval_us = interval_us;
}

static void IMU_RecomputeResolvedRequests(void)
{
    imu_request_set_t resolved;
    imu_report_id_t report_id;
    uint32_t client_idx;

    IMU_ClearRequestSet(&resolved);
    for (report_id = 0; report_id < IMU_REPORT_COUNT; ++report_id) {
        uint8_t enabled = 0U;
        uint32_t winning_interval = 0U;
        for (client_idx = 0; client_idx < IMU_CLIENT_COUNT; ++client_idx) {
            const imu_request_entry_t *entry = &g_imu_runtime.client_requests[client_idx].reports[report_id];
            if (entry->enabled == 0U || entry->interval_us == 0U) {
                continue;
            }
            if (enabled == 0U || entry->interval_us < winning_interval) {
                enabled = 1U;
                winning_interval = entry->interval_us;
            }
        }
        resolved.reports[report_id].enabled = enabled;
        resolved.reports[report_id].interval_us = winning_interval;
    }

    if (resolved.reports[IMU_REPORT_MAG_RAW].enabled != 0U) {
        imu_request_entry_t *mag_cal = &resolved.reports[IMU_REPORT_MAG_CAL];
        uint32_t raw_interval = resolved.reports[IMU_REPORT_MAG_RAW].interval_us;
        if (mag_cal->enabled == 0U || mag_cal->interval_us == 0U || raw_interval < mag_cal->interval_us) {
            mag_cal->enabled = 1U;
            mag_cal->interval_us = raw_interval;
        }
    }

    g_imu_runtime.resolved_request_set = resolved;
}

void IMU_Init(void)
{
    (void)memset(&g_imu_runtime, 0, sizeof(g_imu_runtime));
}

void IMU_RequestSet(imu_client_id_t client_id, const imu_request_set_t *request_set)
{
    if (client_id >= IMU_CLIENT_COUNT || request_set == NULL) {
        return;
    }
    IMU_RUNTIME_ENTER_CRITICAL();
    g_imu_runtime.client_requests[client_id] = *request_set;
    IMU_RecomputeResolvedRequests();
    IMU_RUNTIME_EXIT_CRITICAL();
}

void IMU_BuildControlRequestSet(adcs_mode_t mode, bool mtq_firing, imu_request_set_t *out)
{
    uint32_t control_interval_us = BNO085_REPORT_INTERVAL_US;
    IMU_ClearRequestSet(out);
    if (out == NULL) {
        return;
    }

    switch (mode) {
        case CTRL_MODE_DETUMBLE:
            IMU_SetRequest(out, IMU_REPORT_GYRO, control_interval_us);
            IMU_SetRequest(out, IMU_REPORT_MAG_CAL, control_interval_us);
            break;
        case CTRL_MODE_SPIN_STABLE:
            IMU_SetRequest(out, IMU_REPORT_GYRO, control_interval_us);
            IMU_SetRequest(out, IMU_REPORT_MAG_CAL, control_interval_us);
            break;
        case CTRL_MODE_POINTING:
            IMU_SetRequest(out, IMU_REPORT_GYRO, control_interval_us);
            IMU_SetRequest(out, IMU_REPORT_ROTATION_VECTOR, control_interval_us);
            IMU_SetRequest(out, IMU_REPORT_MAG_CAL, control_interval_us);
            break;
        case CTRL_MODE_IDLE:
        default:
            break;
    }

    if (mtq_firing) {
        IMU_SetRequest(out, IMU_REPORT_GAME_ROTATION_VECTOR, control_interval_us);
        IMU_SetRequest(out, IMU_REPORT_MAG_UNCAL, control_interval_us);
        IMU_SetRequest(out, IMU_REPORT_MAG_RAW, control_interval_us);
    }
}

void IMU_BuildTelemetryBasicRequestSet(imu_request_set_t *out)
{
    uint32_t telemetry_interval_us = BNO085_REPORT_INTERVAL_US * 2U;
    IMU_ClearRequestSet(out);
    if (out == NULL) {
        return;
    }

    IMU_SetRequest(out, IMU_REPORT_ROTATION_VECTOR, telemetry_interval_us);
    IMU_SetRequest(out, IMU_REPORT_GAME_ROTATION_VECTOR, telemetry_interval_us);
    IMU_SetRequest(out, IMU_REPORT_GYRO, telemetry_interval_us);
    IMU_SetRequest(out, IMU_REPORT_MAG_CAL, telemetry_interval_us);
    IMU_SetRequest(out, IMU_REPORT_MAG_UNCAL, telemetry_interval_us);
    IMU_SetRequest(out, IMU_REPORT_MAG_RAW, telemetry_interval_us);
    IMU_SetRequest(out, IMU_REPORT_LINEAR_ACCEL, telemetry_interval_us);
}

void IMU_RequestProfile(imu_client_id_t client_id, imu_profile_id_t profile_id)
{
    imu_request_set_t request_set;
    IMU_ClearRequestSet(&request_set);

    switch (profile_id) {
        case IMU_PROFILE_TELEMETRY_BASIC:
            IMU_BuildTelemetryBasicRequestSet(&request_set);
            break;
        case IMU_PROFILE_CTRL_DETUMBLE:
            IMU_BuildControlRequestSet(CTRL_MODE_DETUMBLE, false, &request_set);
            break;
        case IMU_PROFILE_CTRL_SPIN_STABLE:
            IMU_BuildControlRequestSet(CTRL_MODE_SPIN_STABLE, false, &request_set);
            break;
        case IMU_PROFILE_CTRL_POINTING:
            IMU_BuildControlRequestSet(CTRL_MODE_POINTING, false, &request_set);
            break;
        case IMU_PROFILE_CTRL_DETUMBLE_MTQ_FIRING:
            IMU_BuildControlRequestSet(CTRL_MODE_DETUMBLE, true, &request_set);
            break;
        case IMU_PROFILE_CTRL_SPIN_STABLE_MTQ_FIRING:
            IMU_BuildControlRequestSet(CTRL_MODE_SPIN_STABLE, true, &request_set);
            break;
        case IMU_PROFILE_CTRL_POINTING_MTQ_FIRING:
            IMU_BuildControlRequestSet(CTRL_MODE_POINTING, true, &request_set);
            break;
        case IMU_PROFILE_NONE:
        default:
            break;
    }

    IMU_RequestSet(client_id, &request_set);
}

void IMU_CopyResolvedRequestSet(imu_request_set_t *out)
{
    if (out == NULL) {
        return;
    }
    IMU_RUNTIME_ENTER_CRITICAL();
    *out = g_imu_runtime.resolved_request_set;
    IMU_RUNTIME_EXIT_CRITICAL();
}

void IMU_CopyClientRequestSet(imu_client_id_t client_id, imu_request_set_t *out)
{
    if (client_id >= IMU_CLIENT_COUNT || out == NULL) {
        return;
    }
    IMU_RUNTIME_ENTER_CRITICAL();
    *out = g_imu_runtime.client_requests[client_id];
    IMU_RUNTIME_EXIT_CRITICAL();
}

void IMU_SetAppliedRequest(imu_report_id_t report_id, bool enabled, uint32_t interval_us)
{
    if (report_id >= IMU_REPORT_COUNT) {
        return;
    }
    IMU_RUNTIME_ENTER_CRITICAL();
    g_imu_runtime.applied_request_set.reports[report_id].enabled = enabled ? 1U : 0U;
    g_imu_runtime.applied_request_set.reports[report_id].interval_us = enabled ? interval_us : 0U;
    IMU_RUNTIME_EXIT_CRITICAL();
}

void IMU_CopyAppliedRequestSet(imu_request_set_t *out)
{
    if (out == NULL) {
        return;
    }
    IMU_RUNTIME_ENTER_CRITICAL();
    *out = g_imu_runtime.applied_request_set;
    IMU_RUNTIME_EXIT_CRITICAL();
}

void IMU_PublishSnapshot(const imu_snapshot_t *snapshot)
{
    uint8_t next_index;
    if (snapshot == NULL) {
        return;
    }

    IMU_RUNTIME_ENTER_CRITICAL();
    next_index = (uint8_t)(g_imu_runtime.published_index ^ 1U);
    g_imu_runtime.snapshots[next_index] = *snapshot;
    g_imu_runtime.publish_sequence += 1U;
    g_imu_runtime.snapshots[next_index].publish_sequence = g_imu_runtime.publish_sequence;
    g_imu_runtime.published_index = next_index;
    IMU_RUNTIME_EXIT_CRITICAL();
}

void IMU_ReadSnapshot(imu_snapshot_t *out)
{
    if (out == NULL) {
        return;
    }
    IMU_RUNTIME_ENTER_CRITICAL();
    *out = g_imu_runtime.snapshots[g_imu_runtime.published_index];
    IMU_RUNTIME_EXIT_CRITICAL();
}

const char *IMU_ReportName(imu_report_id_t report_id)
{
    static const char *const k_names[IMU_REPORT_COUNT] = {
        "Rotation Vector",
        "Game Rotation Vector",
        "Gyroscope",
        "Magnetic Field",
        "Magnetic Field Uncal",
        "Raw Magnetometer",
        "Linear Acceleration",
        "Accelerometer",
        "Gravity"
    };

    if (report_id >= IMU_REPORT_COUNT) {
        return "Unknown";
    }
    return k_names[report_id];
}

uint8_t IMU_ReportToFeatureId(imu_report_id_t report_id)
{
    static const uint8_t k_feature_ids[IMU_REPORT_COUNT] = {
        IMU_FEATURE_ROTATION_VECTOR,
        IMU_FEATURE_GAME_ROTATION_VECTOR,
        IMU_FEATURE_GYROSCOPE,
        IMU_FEATURE_MAGNETIC_FIELD,
        IMU_FEATURE_MAGNETIC_FIELD_UNCAL,
        IMU_FEATURE_RAW_MAGNETOMETER,
        IMU_FEATURE_LINEAR_ACCELERATION,
        IMU_FEATURE_ACCELEROMETER,
        IMU_FEATURE_GRAVITY
    };

    if (report_id >= IMU_REPORT_COUNT) {
        return 0U;
    }
    return k_feature_ids[report_id];
}
