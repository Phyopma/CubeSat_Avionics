#ifndef PHOTODIODE_H
#define PHOTODIODE_H

#include "photodiode_runtime.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Photodiode_Init(void);
int Photodiode_ReadRaw(photodiode_sample_t *out, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
