/*
 * teleplot.h
 *
 *  Created on: Nov 28, 2025
 *      Author: phyopyae
 */

#ifndef INC_TELEPLOT_H_
#define INC_TELEPLOT_H_

#include <stdio.h>
#include <stdint.h>
#include "main.h"

extern UART_HandleTypeDef huart2;

void Teleplot_Update(const char *label, float value);

#endif /* INC_TELEPLOT_H_ */
