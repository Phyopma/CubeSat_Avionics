#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void AppRuntime_Init(void);
void AppRuntime_Start(void);
void AppRuntime_RunOnce(void);
void AppRuntime_OnControlTickFromISR(void);

#ifdef __cplusplus
}
#endif

#endif
