#include "power_runtime.h"

#include <string.h>

#if defined(USE_HAL_DRIVER)
#include "FreeRTOS.h"
#include "task.h"
#define POWER_RUNTIME_ENTER_CRITICAL() taskENTER_CRITICAL()
#define POWER_RUNTIME_EXIT_CRITICAL()  taskEXIT_CRITICAL()
#else
#define POWER_RUNTIME_ENTER_CRITICAL() ((void)0)
#define POWER_RUNTIME_EXIT_CRITICAL()  ((void)0)
#endif

typedef struct {
    power_snapshot_t snapshots[2];
    uint8_t published_index;
    uint32_t publish_sequence;
} power_runtime_state_t;

static power_runtime_state_t g_power_runtime;

void Power_Init(void)
{
    (void)memset(&g_power_runtime, 0, sizeof(g_power_runtime));
}

void Power_PublishSnapshot(const power_snapshot_t *snapshot)
{
    uint8_t next_index;
    if (snapshot == NULL) {
        return;
    }

    POWER_RUNTIME_ENTER_CRITICAL();
    next_index = (uint8_t)(g_power_runtime.published_index ^ 1U);
    g_power_runtime.snapshots[next_index] = *snapshot;
    g_power_runtime.publish_sequence += 1U;
    g_power_runtime.snapshots[next_index].publish_sequence = g_power_runtime.publish_sequence;
    g_power_runtime.published_index = next_index;
    POWER_RUNTIME_EXIT_CRITICAL();
}

void Power_ReadSnapshot(power_snapshot_t *out)
{
    if (out == NULL) {
        return;
    }
    POWER_RUNTIME_ENTER_CRITICAL();
    *out = g_power_runtime.snapshots[g_power_runtime.published_index];
    POWER_RUNTIME_EXIT_CRITICAL();
}
