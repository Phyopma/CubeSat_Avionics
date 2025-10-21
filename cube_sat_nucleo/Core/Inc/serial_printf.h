/*
 * serial_printf.h
 *
 *  Created on: Oct 20, 2025
 *      Author: phyopyae
 */

#ifndef INC_SERIAL_PRINTF_H_
#define INC_SERIAL_PRINTF_H_

#include "stm32l4xx_hal.h"
#include <stdio.h>
extern UART_HandleTypeDef huart2;

int __io_putchar(int ch);
int _write(int file, char *ptr, int len);

#endif /* INC_SERIAL_PRINTF_H_ */
