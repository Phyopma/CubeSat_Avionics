/*
 * serial_log_dma.c
 *
 *  Created on: Oct 21, 2025
 *      Author: phyopyae
 */



#include "serial_log_dma.h"
static UART_HandleTypeDef *g_huart = NULL;
static volatile uint8_t g_tx_busy = 0;
static char g_buf[LOG_BUF_SZ];

void serial_log_init(UART_HandleTypeDef *huart) { g_huart = huart; }  // [web:134]

void log_printf_dma(const char *fmt, ...) {
    if (!g_huart || g_tx_busy) return;                                 // [web:295]
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(g_buf, sizeof(g_buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;                                                 // [web:139]
    if ((size_t)n + 2 < sizeof(g_buf)) { g_buf[n++] = '\r'; g_buf[n++] = '\n'; } // [web:139]
    g_tx_busy = 1;                                                     // [web:295]
    if (HAL_UART_Transmit_DMA(g_huart, (uint8_t*)g_buf, (uint16_t)n) != HAL_OK) {
        g_tx_busy = 0;                                                 // [web:295]
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == g_huart) g_tx_busy = 0;                               // [web:295]
}
