#include "manual_validation.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static float ManualValidation_ClampCurrent(float amps)
{
    if (amps > MANUAL_VALIDATION_MAX_CURRENT_AMPS)
    {
        return MANUAL_VALIDATION_MAX_CURRENT_AMPS;
    }
    if (amps < -MANUAL_VALIDATION_MAX_CURRENT_AMPS)
    {
        return -MANUAL_VALIDATION_MAX_CURRENT_AMPS;
    }
    return amps;
}

static uint32_t ManualValidation_ClampPeriod(uint32_t period_ms)
{
    if (period_ms < MANUAL_VALIDATION_MIN_PERIOD_MS)
    {
        return MANUAL_VALIDATION_MIN_PERIOD_MS;
    }
    if (period_ms > MANUAL_VALIDATION_MAX_PERIOD_MS)
    {
        return MANUAL_VALIDATION_MAX_PERIOD_MS;
    }
    return period_ms;
}

static const char *ManualValidation_AxisName(manual_validation_axis_t axis)
{
    switch (axis)
    {
    case MANUAL_VALIDATION_AXIS_X:
        return "X";
    case MANUAL_VALIDATION_AXIS_Y:
        return "Y";
    case MANUAL_VALIDATION_AXIS_Z:
        return "Z";
    default:
        return "?";
    }
}

static const char *ManualValidation_ModeName(manual_validation_mode_t mode)
{
    switch (mode)
    {
    case MANUAL_VALIDATION_MODE_OFF:
        return "OFF";
    case MANUAL_VALIDATION_MODE_STEP:
        return "STEP";
    case MANUAL_VALIDATION_MODE_SQUARE:
        return "SQUARE";
    case MANUAL_VALIDATION_MODE_PULSE:
        return "PULSE";
    default:
        return "?";
    }
}

static void ManualValidation_SetMessage(manual_validation_command_result_t *result, manual_validation_command_code_t code, const char *message)
{
    if (result == NULL)
    {
        return;
    }

    result->code = code;
    if (message == NULL)
    {
        result->message[0] = '\0';
        return;
    }

    (void)snprintf(result->message, sizeof(result->message), "%s", message);
}

static void ManualValidation_Trim(char *text)
{
    size_t start = 0U;
    size_t len;

    if (text == NULL)
    {
        return;
    }

    len = strlen(text);
    while (start < len && isspace((unsigned char)text[start]) != 0)
    {
        start++;
    }
    while (len > start && isspace((unsigned char)text[len - 1U]) != 0)
    {
        len--;
    }

    if (start > 0U)
    {
        memmove(text, text + start, len - start);
    }
    text[len - start] = '\0';
}

void ManualValidation_Init(manual_validation_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->owner = ACTUATOR_OWNER_MANUAL_VALIDATION;
    state->armed = 0U;
    state->axis = MANUAL_VALIDATION_AXIS_Z;
    state->mode = MANUAL_VALIDATION_MODE_SQUARE;
    state->current_amps = MANUAL_VALIDATION_DEFAULT_CURRENT_AMPS;
    state->period_ms = MANUAL_VALIDATION_DEFAULT_PERIOD_MS;
}

int ManualValidation_IsOverrideActive(const manual_validation_state_t *state)
{
    if (state == NULL)
    {
        return 0;
    }

    return (state->owner == ACTUATOR_OWNER_MANUAL_VALIDATION &&
            state->armed != 0U &&
            state->mode != MANUAL_VALIDATION_MODE_OFF);
}

void ManualValidation_FormatStatus(const manual_validation_state_t *state, char *buffer, size_t buffer_size)
{
    const char *owner_name;

    if (buffer == NULL || buffer_size == 0U)
    {
        return;
    }

    if (state == NULL)
    {
        (void)snprintf(buffer, buffer_size, "manual_validation unavailable");
        return;
    }

    owner_name = (state->owner == ACTUATOR_OWNER_MANUAL_VALIDATION) ? "MANUAL_VALIDATION" : "ADCS";
    (void)snprintf(
        buffer,
        buffer_size,
        "owner=%s armed=%u axis=%s mode=%s current_ma=%.1f period_ms=%lu active=%u",
        owner_name,
        (unsigned int)state->armed,
        ManualValidation_AxisName(state->axis),
        ManualValidation_ModeName(state->mode),
        (double)(state->current_amps * 1000.0f),
        (unsigned long)state->period_ms,
        (unsigned int)ManualValidation_IsOverrideActive(state));
}

void ManualValidation_ComputeOutput(const manual_validation_state_t *state, uint32_t now_ms, manual_validation_output_t *output)
{
    float command = 0.0f;
    uint32_t half_period_ms;

    if (output == NULL)
    {
        return;
    }

    output->target_current_x = 0.0f;
    output->target_current_y = 0.0f;
    output->target_current_z = 0.0f;

    if (!ManualValidation_IsOverrideActive(state))
    {
        return;
    }

    command = state->current_amps;
    if (state->mode == MANUAL_VALIDATION_MODE_SQUARE)
    {
        half_period_ms = state->period_ms / 2U;
        if (half_period_ms == 0U)
        {
            half_period_ms = 1U;
        }
        if (((now_ms / half_period_ms) & 0x1U) != 0U)
        {
            command = -command;
        }
    }
    else if (state->mode == MANUAL_VALIDATION_MODE_PULSE)
    {
        half_period_ms = state->period_ms / 2U;
        if (half_period_ms == 0U)
        {
            half_period_ms = 1U;
        }
        if (((now_ms / half_period_ms) & 0x1U) != 0U)
        {
            command = 0.0f;
        }
    }

    switch (state->axis)
    {
    case MANUAL_VALIDATION_AXIS_X:
        output->target_current_x = command;
        break;
    case MANUAL_VALIDATION_AXIS_Y:
        output->target_current_y = command;
        break;
    case MANUAL_VALIDATION_AXIS_Z:
        output->target_current_z = command;
        break;
    default:
        break;
    }
}

manual_validation_command_result_t ManualValidation_ApplyCommand(manual_validation_state_t *state, const char *line)
{
    manual_validation_command_result_t result;
    char buffer[96];
    char keyword[32];
    char value[32];
    char *end_ptr = NULL;

    ManualValidation_SetMessage(&result, MANUAL_VALIDATION_CMD_ERROR, "invalid command");
    if (state == NULL || line == NULL)
    {
        return result;
    }

    (void)snprintf(buffer, sizeof(buffer), "%s", line);
    ManualValidation_Trim(buffer);

    if (strcmp(buffer, "val arm") == 0)
    {
        state->armed = 1U;
        ManualValidation_SetMessage(&result, MANUAL_VALIDATION_CMD_OK, "manual validation armed");
        return result;
    }
    if (strcmp(buffer, "val disarm") == 0)
    {
        state->armed = 0U;
        ManualValidation_SetMessage(&result, MANUAL_VALIDATION_CMD_OK, "manual validation disarmed");
        return result;
    }
    if (strcmp(buffer, "val off") == 0)
    {
        state->mode = MANUAL_VALIDATION_MODE_OFF;
        ManualValidation_SetMessage(&result, MANUAL_VALIDATION_CMD_OK, "manual validation off");
        return result;
    }

    keyword[0] = '\0';
    value[0] = '\0';
    if (sscanf(buffer, "val %31s %31s", keyword, value) != 2)
    {
        ManualValidation_SetMessage(&result, MANUAL_VALIDATION_CMD_ERROR, "expected 'val <key> <value>'");
        return result;
    }

    if (strcmp(keyword, "axis") == 0)
    {
        if (strcmp(value, "x") == 0)
        {
            state->axis = MANUAL_VALIDATION_AXIS_X;
        }
        else if (strcmp(value, "y") == 0)
        {
            state->axis = MANUAL_VALIDATION_AXIS_Y;
        }
        else if (strcmp(value, "z") == 0)
        {
            state->axis = MANUAL_VALIDATION_AXIS_Z;
        }
        else
        {
            ManualValidation_SetMessage(&result, MANUAL_VALIDATION_CMD_ERROR, "axis must be x, y, or z");
            return result;
        }
        ManualValidation_SetMessage(&result, MANUAL_VALIDATION_CMD_OK, "axis updated");
        return result;
    }

    if (strcmp(keyword, "mode") == 0)
    {
        if (strcmp(value, "step") == 0)
        {
            state->mode = MANUAL_VALIDATION_MODE_STEP;
        }
        else if (strcmp(value, "square") == 0)
        {
            state->mode = MANUAL_VALIDATION_MODE_SQUARE;
        }
        else if (strcmp(value, "pulse") == 0)
        {
            state->mode = MANUAL_VALIDATION_MODE_PULSE;
        }
        else
        {
            ManualValidation_SetMessage(&result, MANUAL_VALIDATION_CMD_ERROR, "mode must be step, square, or pulse");
            return result;
        }
        ManualValidation_SetMessage(&result, MANUAL_VALIDATION_CMD_OK, "mode updated");
        return result;
    }

    if (strcmp(keyword, "current_ma") == 0)
    {
        float current_ma = strtof(value, &end_ptr);
        if (end_ptr == value || *end_ptr != '\0')
        {
            ManualValidation_SetMessage(&result, MANUAL_VALIDATION_CMD_ERROR, "current_ma must be numeric");
            return result;
        }
        state->current_amps = ManualValidation_ClampCurrent(current_ma * 0.001f);
        ManualValidation_SetMessage(&result, MANUAL_VALIDATION_CMD_OK, "current updated");
        return result;
    }

    if (strcmp(keyword, "period_ms") == 0)
    {
        unsigned long parsed = strtoul(value, &end_ptr, 10);
        if (end_ptr == value || *end_ptr != '\0')
        {
            ManualValidation_SetMessage(&result, MANUAL_VALIDATION_CMD_ERROR, "period_ms must be numeric");
            return result;
        }
        state->period_ms = ManualValidation_ClampPeriod((uint32_t)parsed);
        ManualValidation_SetMessage(&result, MANUAL_VALIDATION_CMD_OK, "period updated");
        return result;
    }

    ManualValidation_SetMessage(&result, MANUAL_VALIDATION_CMD_ERROR, "unknown val command");
    return result;
}
