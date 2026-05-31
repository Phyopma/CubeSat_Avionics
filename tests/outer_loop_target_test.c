#include "outer_loop_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void test_sun_target_vector_drives_pointing_error(void)
{
    adcs_sensor_input_t input;
    adcs_output_t output;
    vec3_t tau_raw;

    memset(&input, 0, sizeof(input));
    memset(&output, 0, sizeof(output));

    OuterLoop_Init();
    OuterLoop_SetMode(CTRL_MODE_POINTING);

    input.mag_field = (vec3_t){0.0f, 0.0f, 30.0e-6f};
    input.orientation = (quat_t){1.0f, 0.0f, 0.0f, 0.0f};
    input.pointing_target.source = POINTING_TARGET_SUN;
    input.pointing_target.vector_valid = 1U;
    input.pointing_target.body_axis = (vec3_t){0.0f, 0.0f, 1.0f};
    input.pointing_target.target_body = (vec3_t){0.0f, 1.0f, 0.0f};

    OuterLoop_Update(&input, &output, 0.01f);
    OuterLoop_GetLastTorqueRaw(&tau_raw);

    assert(tau_raw.x < 0.0f);
    assert(fabsf(tau_raw.y) < 1e-7f);
    assert(output.dipole_request.y < 0.0f);
}

static void test_invalid_sun_target_falls_back_to_safe_zero_when_stationary(void)
{
    adcs_sensor_input_t input;
    adcs_output_t output;

    memset(&input, 0, sizeof(input));
    memset(&output, 0, sizeof(output));

    OuterLoop_Init();
    OuterLoop_SetMode(CTRL_MODE_POINTING);

    input.mag_field = (vec3_t){0.0f, 0.0f, 30.0e-6f};
    input.orientation = (quat_t){1.0f, 0.0f, 0.0f, 0.0f};
    input.pointing_target.source = POINTING_TARGET_SUN;
    input.pointing_target.vector_valid = 0U;

    OuterLoop_Update(&input, &output, 0.01f);

    assert(fabsf(output.dipole_request.x) < 1e-9f);
    assert(fabsf(output.dipole_request.y) < 1e-9f);
    assert(fabsf(output.dipole_request.z) < 1e-9f);
}

int main(void)
{
    test_sun_target_vector_drives_pointing_error();
    test_invalid_sun_target_falls_back_to_safe_zero_when_stationary();
    puts("outer_loop_target_test: PASS");
    return 0;
}
