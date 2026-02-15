#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "serial_log_dma.h"

// Call this to plot a value
void Teleplot_Update(const char *label, float value)
{
    log_printf_async(">%s:%.2f", label, value);
}
