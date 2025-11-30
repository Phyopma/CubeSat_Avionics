#include <stdio.h>
#include <stdint.h>
#include "main.h"

extern UART_HandleTypeDef huart2;

// Call this to plot a value
void Teleplot_Update(const char *label, float value)
{
    char buffer[64];
    // Format: >Label:123.45\n
    int len = sprintf(buffer, ">%s:%.2f\n", label, value);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 10);
}
