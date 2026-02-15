/*
 * serial_log_dma.c
 *
 *  Created on: Oct 21, 2025
 *      Author: phyopyae
 */

#include "serial_log_dma.h"
#include <stdbool.h>
static UART_HandleTypeDef *g_huart = NULL;
static volatile uint8_t g_tx_busy = 0;

#define LOG_QUEUE_DEPTH 16
static char g_queue[LOG_QUEUE_DEPTH][LOG_BUF_SZ];
static volatile uint8_t g_q_head = 0;
static volatile uint8_t g_q_tail = 0;
static volatile uint8_t g_q_count = 0;
static volatile uint32_t g_drop_count = 0;
static char g_tx_buf[LOG_BUF_SZ];
static volatile uint8_t g_kick_tx = 0U;

void serial_log_init(UART_HandleTypeDef *huart) { g_huart = huart; }

static bool queue_push(const char *msg, size_t n)
{
    __disable_irq();
    if (g_q_count >= LOG_QUEUE_DEPTH) {
        __enable_irq();
        g_drop_count++;
        return false;
    }
    size_t copy_n = (n >= LOG_BUF_SZ) ? (LOG_BUF_SZ - 1U) : n;
    memcpy(g_queue[g_q_tail], msg, copy_n);
    g_queue[g_q_tail][copy_n] = '\0';
    g_q_tail = (uint8_t)((g_q_tail + 1U) % LOG_QUEUE_DEPTH);
    g_q_count++;
    __enable_irq();
    return true;
}

static bool queue_pop(char *out, size_t out_sz)
{
    __disable_irq();
    if (g_q_count == 0 || out_sz == 0U) {
        __enable_irq();
        return false;
    }
    size_t n = strnlen(g_queue[g_q_head], LOG_BUF_SZ);
    if (n >= out_sz) {
        n = out_sz - 1U;
    }
    memcpy(out, g_queue[g_q_head], n);
    out[n] = '\0';
    g_q_head = (uint8_t)((g_q_head + 1U) % LOG_QUEUE_DEPTH);
    g_q_count--;
    __enable_irq();
    return true;
}

void log_printf_async(const char *fmt, ...)
{
    if (!g_huart) {
        return;
    }
    char msg[LOG_BUF_SZ];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    if ((size_t)n + 2U < sizeof(msg)) {
        msg[n++] = '\r';
        msg[n++] = '\n';
        msg[n] = '\0';
    }
    (void)queue_push(msg, (size_t)n);
}

void log_printf_dma(const char *fmt, ...)
{
    char msg[LOG_BUF_SZ];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    if ((size_t)n + 2U < sizeof(msg)) {
        msg[n++] = '\r';
        msg[n++] = '\n';
        msg[n] = '\0';
    }
    if (queue_push(msg, (size_t)n)) {
        serial_log_process_tx();
    }
}

void serial_log_process_tx(void)
{
    if (!g_huart || g_tx_busy) {
        return;
    }
    g_kick_tx = 0U;
    if (!queue_pop(g_tx_buf, sizeof(g_tx_buf))) {
        return;
    }
    g_tx_busy = 1;
    g_kick_tx = 0U;
    if (HAL_UART_Transmit_DMA(g_huart, (uint8_t *)g_tx_buf, (uint16_t)strnlen(g_tx_buf, sizeof(g_tx_buf))) != HAL_OK) {
        g_tx_busy = 0;
    }
}

uint32_t serial_log_drop_count(void)
{
    return g_drop_count;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == g_huart) {
        g_tx_busy = 0;
        g_kick_tx = 1U;
    }
}
