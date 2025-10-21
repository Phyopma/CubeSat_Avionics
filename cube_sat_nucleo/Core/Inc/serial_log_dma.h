/*
 * serial_log_dma.h
 *
 *  Created on: Oct 21, 2025
 *      Author: phyopyae
 */

#ifndef INC_SERIAL_LOG_DMA_H_
#define INC_SERIAL_LOG_DMA_H_

#include "stm32l4xx_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#ifndef LOG_BUF_SZ
#define LOG_BUF_SZ 256
#endif


void serial_log_init(UART_HandleTypeDef *huart);
void log_printf_dma(const char *fmt, ...);


#endif /* INC_SERIAL_LOG_DMA_H_ */
