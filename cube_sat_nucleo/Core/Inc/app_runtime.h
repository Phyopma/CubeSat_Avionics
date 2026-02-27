#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

void AppRuntime_Init(void);
void AppRuntime_Start(void);
void AppRuntime_RunOnce(void);
void AppRuntime_OnControlTickFromISR(void);

// Exported for UART ISR to notify ADCS task of new sim packet
extern TaskHandle_t g_adcs_task_handle;

#ifdef __cplusplus
}
#endif

#endif
