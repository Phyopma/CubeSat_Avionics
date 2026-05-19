#ifndef IMU_RUNTIME_H
#define IMU_RUNTIME_H

#include "outer_loop_control.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IMU_REPORT_ROTATION_VECTOR = 0,
    IMU_REPORT_GAME_ROTATION_VECTOR,
    IMU_REPORT_GYRO,
    IMU_REPORT_MAG_CAL,
    IMU_REPORT_MAG_UNCAL,
    IMU_REPORT_MAG_RAW,
    IMU_REPORT_LINEAR_ACCEL,
    IMU_REPORT_ACCEL,
    IMU_REPORT_GRAVITY,
    IMU_REPORT_COUNT
} imu_report_id_t;

typedef enum {
    IMU_CLIENT_CONTROL = 0,
    IMU_CLIENT_COMMS,
    IMU_CLIENT_COUNT
} imu_client_id_t;

typedef enum {
    IMU_PROFILE_NONE = 0,
    IMU_PROFILE_TELEMETRY_BASIC,
    IMU_PROFILE_CTRL_DETUMBLE,
    IMU_PROFILE_CTRL_SPIN_STABLE,
    IMU_PROFILE_CTRL_POINTING,
    IMU_PROFILE_CTRL_DETUMBLE_MTQ_FIRING,
    IMU_PROFILE_CTRL_SPIN_STABLE_MTQ_FIRING,
    IMU_PROFILE_CTRL_POINTING_MTQ_FIRING
} imu_profile_id_t;

typedef struct {
    uint8_t enabled;
    uint32_t interval_us;
} imu_request_entry_t;

typedef struct {
    imu_request_entry_t reports[IMU_REPORT_COUNT];
} imu_request_set_t;

typedef struct {
    float x;
    float y;
    float z;
} imu_vec3f_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} imu_raw_vec3_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
    float accuracy_rad;
} imu_quat_t;

typedef struct {
    uint8_t valid;
    uint8_t enabled;
    uint8_t status;
    uint8_t accuracy;
    uint32_t local_timestamp_ms;
    uint32_t sensor_timestamp_us;
    uint32_t sample_sequence;
} imu_report_meta_t;

typedef struct {
    imu_report_meta_t meta;
    imu_quat_t value;
} imu_quat_slot_t;

typedef struct {
    imu_report_meta_t meta;
    imu_vec3f_t value;
} imu_vec3_slot_t;

typedef struct {
    imu_report_meta_t meta;
    imu_raw_vec3_t value;
} imu_raw_vec3_slot_t;

typedef struct {
    uint32_t publish_sequence;
    uint32_t publish_timestamp_ms;
    imu_quat_slot_t rotation_vector;
    imu_quat_slot_t game_rotation_vector;
    imu_vec3_slot_t gyro;
    imu_vec3_slot_t magnetometer_cal;
    imu_vec3_slot_t magnetometer_uncal;
    imu_raw_vec3_slot_t magnetometer_raw;
    imu_vec3_slot_t linear_accel;
    imu_vec3_slot_t accel;
    imu_vec3_slot_t gravity;
} imu_snapshot_t;

void IMU_Init(void);
void IMU_RequestSet(imu_client_id_t client_id, const imu_request_set_t *request_set);
void IMU_RequestProfile(imu_client_id_t client_id, imu_profile_id_t profile_id);
void IMU_BuildControlRequestSet(adcs_mode_t mode, bool mtq_firing, imu_request_set_t *out);
void IMU_BuildTelemetryBasicRequestSet(imu_request_set_t *out);
void IMU_CopyResolvedRequestSet(imu_request_set_t *out);
void IMU_CopyClientRequestSet(imu_client_id_t client_id, imu_request_set_t *out);
void IMU_SetAppliedRequest(imu_report_id_t report_id, bool enabled, uint32_t interval_us);
void IMU_CopyAppliedRequestSet(imu_request_set_t *out);
void IMU_PublishSnapshot(const imu_snapshot_t *snapshot);
void IMU_ReadSnapshot(imu_snapshot_t *out);
const char *IMU_ReportName(imu_report_id_t report_id);
uint8_t IMU_ReportToFeatureId(imu_report_id_t report_id);

#ifdef __cplusplus
}
#endif

#endif
