#ifndef MANUAL_VALIDATION_H
#define MANUAL_VALIDATION_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MANUAL_VALIDATION_DEFAULT_PERIOD_MS 500U
#define MANUAL_VALIDATION_MIN_PERIOD_MS 20U
#define MANUAL_VALIDATION_MAX_PERIOD_MS 60000U
// 3.3 V / 28 ohm keeps manual excitation within the nominal hardware limit.
#define MANUAL_VALIDATION_MAX_CURRENT_AMPS 0.11785714f
#define MANUAL_VALIDATION_DEFAULT_CURRENT_AMPS 0.10f

typedef enum
{
    ACTUATOR_OWNER_ADCS = 0,
    ACTUATOR_OWNER_MANUAL_VALIDATION = 1
} actuator_owner_t;

typedef enum
{
    MANUAL_VALIDATION_AXIS_X = 0,
    MANUAL_VALIDATION_AXIS_Y = 1,
    MANUAL_VALIDATION_AXIS_Z = 2
} manual_validation_axis_t;

typedef enum
{
    MANUAL_VALIDATION_MODE_OFF = 0,
    MANUAL_VALIDATION_MODE_STEP = 1,
    MANUAL_VALIDATION_MODE_SQUARE = 2,
    MANUAL_VALIDATION_MODE_PULSE = 3
} manual_validation_mode_t;

typedef enum
{
    MANUAL_VALIDATION_CMD_OK = 0,
    MANUAL_VALIDATION_CMD_ERROR = 1
} manual_validation_command_code_t;

typedef struct
{
    manual_validation_command_code_t code;
    char message[96];
} manual_validation_command_result_t;

typedef struct
{
    actuator_owner_t owner;
    uint8_t armed;
    manual_validation_axis_t axis;
    manual_validation_mode_t mode;
    float current_amps;
    uint32_t period_ms;
} manual_validation_state_t;

typedef struct
{
    float target_current_x;
    float target_current_y;
    float target_current_z;
} manual_validation_output_t;

void ManualValidation_Init(manual_validation_state_t *state);
manual_validation_command_result_t ManualValidation_ApplyCommand(manual_validation_state_t *state, const char *line);
void ManualValidation_ComputeOutput(const manual_validation_state_t *state, uint32_t now_ms, manual_validation_output_t *output);
int ManualValidation_IsOverrideActive(const manual_validation_state_t *state);
void ManualValidation_FormatStatus(const manual_validation_state_t *state, char *buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif
