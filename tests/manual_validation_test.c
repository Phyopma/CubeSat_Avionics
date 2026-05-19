#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "manual_validation.h"

static int nearly_equal(float a, float b)
{
    return fabsf(a - b) < 1e-6f;
}

static void test_defaults(void)
{
    manual_validation_state_t state;
    ManualValidation_Init(&state);

    assert(state.owner == ACTUATOR_OWNER_MANUAL_VALIDATION);
    assert(state.armed == 0U);
    assert(state.axis == MANUAL_VALIDATION_AXIS_Z);
    assert(state.mode == MANUAL_VALIDATION_MODE_SQUARE);
    assert(nearly_equal(state.current_amps, 0.10f));
    assert(state.period_ms == 500U);
    assert(ManualValidation_IsOverrideActive(&state) == 0);
}

static void test_command_parser_and_status(void)
{
    manual_validation_state_t state;
    manual_validation_command_result_t result;
    char status[160];

    ManualValidation_Init(&state);

    result = ManualValidation_ApplyCommand(&state, "val axis z");
    assert(result.code == MANUAL_VALIDATION_CMD_OK);
    assert(state.axis == MANUAL_VALIDATION_AXIS_Z);

    result = ManualValidation_ApplyCommand(&state, "val mode square");
    assert(result.code == MANUAL_VALIDATION_CMD_OK);
    assert(state.mode == MANUAL_VALIDATION_MODE_SQUARE);

    result = ManualValidation_ApplyCommand(&state, "val mode pulse");
    assert(result.code == MANUAL_VALIDATION_CMD_OK);
    assert(state.mode == MANUAL_VALIDATION_MODE_PULSE);

    result = ManualValidation_ApplyCommand(&state, "val current_ma 80");
    assert(result.code == MANUAL_VALIDATION_CMD_OK);
    assert(nearly_equal(state.current_amps, 0.08f));

    result = ManualValidation_ApplyCommand(&state, "val period_ms 250");
    assert(result.code == MANUAL_VALIDATION_CMD_OK);
    assert(state.period_ms == 250U);

    result = ManualValidation_ApplyCommand(&state, "val arm");
    assert(result.code == MANUAL_VALIDATION_CMD_OK);
    assert(state.armed == 1U);
    assert(ManualValidation_IsOverrideActive(&state) == 1);

    ManualValidation_FormatStatus(&state, status, sizeof(status));
    assert(strstr(status, "owner=MANUAL_VALIDATION") != NULL);
    assert(strstr(status, "armed=1") != NULL);
    assert(strstr(status, "axis=Z") != NULL);
    assert(strstr(status, "mode=PULSE") != NULL);
}

static void test_output_generation(void)
{
    manual_validation_state_t state;
    manual_validation_output_t output;

    ManualValidation_Init(&state);
    (void)ManualValidation_ApplyCommand(&state, "val axis y");
    (void)ManualValidation_ApplyCommand(&state, "val mode step");
    (void)ManualValidation_ApplyCommand(&state, "val current_ma 50");
    (void)ManualValidation_ApplyCommand(&state, "val disarm");

    ManualValidation_ComputeOutput(&state, 0U, &output);
    assert(nearly_equal(output.target_current_x, 0.0f));
    assert(nearly_equal(output.target_current_y, 0.0f));
    assert(nearly_equal(output.target_current_z, 0.0f));

    (void)ManualValidation_ApplyCommand(&state, "val arm");
    ManualValidation_ComputeOutput(&state, 0U, &output);
    assert(nearly_equal(output.target_current_y, 0.05f));

    (void)ManualValidation_ApplyCommand(&state, "val mode square");
    (void)ManualValidation_ApplyCommand(&state, "val period_ms 200");
    ManualValidation_ComputeOutput(&state, 0U, &output);
    assert(nearly_equal(output.target_current_y, 0.05f));
    ManualValidation_ComputeOutput(&state, 100U, &output);
    assert(nearly_equal(output.target_current_y, -0.05f));
    ManualValidation_ComputeOutput(&state, 200U, &output);
    assert(nearly_equal(output.target_current_y, 0.05f));

    (void)ManualValidation_ApplyCommand(&state, "val mode pulse");
    ManualValidation_ComputeOutput(&state, 0U, &output);
    assert(nearly_equal(output.target_current_y, 0.05f));
    ManualValidation_ComputeOutput(&state, 100U, &output);
    assert(nearly_equal(output.target_current_y, 0.0f));
    ManualValidation_ComputeOutput(&state, 200U, &output);
    assert(nearly_equal(output.target_current_y, 0.05f));
}

static void test_safety_and_clamping(void)
{
    manual_validation_state_t state;
    manual_validation_command_result_t result;
    manual_validation_output_t output;

    ManualValidation_Init(&state);
    result = ManualValidation_ApplyCommand(&state, "val current_ma 999");
    assert(result.code == MANUAL_VALIDATION_CMD_OK);
    assert(nearly_equal(state.current_amps, MANUAL_VALIDATION_MAX_CURRENT_AMPS));

    (void)ManualValidation_ApplyCommand(&state, "val mode step");
    (void)ManualValidation_ApplyCommand(&state, "val arm");
    ManualValidation_ComputeOutput(&state, 0U, &output);
    assert(nearly_equal(output.target_current_z, MANUAL_VALIDATION_MAX_CURRENT_AMPS));

    result = ManualValidation_ApplyCommand(&state, "val disarm");
    assert(result.code == MANUAL_VALIDATION_CMD_OK);
    assert(ManualValidation_IsOverrideActive(&state) == 0);
    ManualValidation_ComputeOutput(&state, 0U, &output);
    assert(nearly_equal(output.target_current_z, 0.0f));

    result = ManualValidation_ApplyCommand(&state, "val off");
    assert(result.code == MANUAL_VALIDATION_CMD_OK);
    assert(state.mode == MANUAL_VALIDATION_MODE_OFF);
}

int main(void)
{
    test_defaults();
    test_command_parser_and_status();
    test_output_generation();
    test_safety_and_clamping();
    return 0;
}
